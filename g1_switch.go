package main

import (
	"context"
	"fmt"

	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// g1-mode exposes the G1's robot modes as a Viam switch so an operator can set
// them from the control UI (a labeled dropdown) instead of hand-typing
// DoCommand JSON on the g1 generic component.
//
// The switch positions are a curated subset of the FSM states that make sense
// as "pick one" modes. Diagnostics (try_fsm, set_speed_mode, arm gestures) stay
// on the g1 generic component — this switch is only for selecting a running mode.
var g1SwitchModel = resource.NewModel("erh", "viam-unitree", "g1-mode")

type g1ModeConfig struct {
	NetworkInterface string `json:"network_interface"`
}

func (c *g1ModeConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

// switchModes is the ordered set of positions this switch exposes. The slice
// index IS the switch position; label is what the UI shows. fsmID is the FSM
// state used to read the current position back in GetPosition. apply drives the
// robot into the mode via the sport service.
var switchModes = []struct {
	label string
	fsmID int
	apply func(ctx context.Context, s *g1Switch) error
}{
	{"zero_torque", FsmZeroTorque, func(ctx context.Context, s *g1Switch) error { return s.loco.SetFsmID(FsmZeroTorque) }},
	{"damp", FsmDamp, func(ctx context.Context, s *g1Switch) error { return s.loco.SetFsmID(FsmDamp) }},
	{"sit", FsmSit, func(ctx context.Context, s *g1Switch) error { return s.loco.SetFsmID(FsmSit) }},
	// "stand" keeps the arms engaged when it comes from walk mode; see standUp.
	{"stand", FsmStandUp, func(ctx context.Context, s *g1Switch) error { return standUp(ctx, s.loco, s.logger) }},
	// "walk" runs ready_to_move (from power-up: Damp → StandUp → Start) and
	// lands the FSM in Regular mode (200), which takes Move commands and keeps
	// walking while arm_sdk holds the arms.
	{"walk", FsmStart, func(ctx context.Context, s *g1Switch) error {
		_, err := runReadyToMove(ctx, s.loco, s.logger, false)
		return err
	}},
}

func init() {
	resource.RegisterComponent(toggleswitch.API, g1SwitchModel,
		resource.Registration[toggleswitch.Switch, *g1ModeConfig]{
			Constructor: newG1Switch,
		})
}

type g1Switch struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	loco   *LocoClient
}

func newG1Switch(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (toggleswitch.Switch, error) {
	cfg, err := resource.NativeConfig[*g1ModeConfig](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	loco, err := NewLocoClient()
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("loco client: %w", err)
	}

	return &g1Switch{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		loco:   loco,
	}, nil
}

func (s *g1Switch) SetPosition(ctx context.Context, position uint32, extra map[string]interface{}) error {
	if int(position) >= len(switchModes) {
		return fmt.Errorf("position %d out of range [0,%d)", position, len(switchModes))
	}
	m := switchModes[position]
	s.logger.Infof("g1-mode: SetPosition %d (%s)", position, m.label)
	// Stand and walk manage arm_sdk themselves and keep the arms engaged where
	// they can. The other positions take torque away from the robot, so the
	// arms are handed back first.
	if !armsStayEngaged(m.fsmID) {
		if err := releaseArmSDK(); err != nil {
			return fmt.Errorf("release arm_sdk: %w", err)
		}
	}
	if err := m.apply(ctx, s); err != nil {
		s.logger.Warnf("g1-mode: %s failed: %v", m.label, err)
		return err
	}
	return nil
}

func (s *g1Switch) GetPosition(ctx context.Context, extra map[string]interface{}) (uint32, error) {
	fsmID, err := s.loco.GetFsmID()
	if err != nil {
		return 0, fmt.Errorf("read FSM id: %w", err)
	}
	if fsmID == FsmRun {
		// Running mode also walks (set from the remote, or by older versions
		// of this module); show it as "walk".
		fsmID = FsmStart
	}
	for i, m := range switchModes {
		if m.fsmID == fsmID {
			return uint32(i), nil
		}
	}
	return 0, fmt.Errorf("current FSM id %d does not map to a known switch position", fsmID)
}

func (s *g1Switch) GetNumberOfPositions(ctx context.Context, extra map[string]interface{}) (uint32, []string, error) {
	labels := make([]string, len(switchModes))
	for i, m := range switchModes {
		labels[i] = m.label
	}
	return uint32(len(switchModes)), labels, nil
}

func (s *g1Switch) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *g1Switch) Close(ctx context.Context) error {
	if s.loco != nil {
		s.loco.Close()
		s.loco = nil
	}
	ShutdownDDS()
	return nil
}
