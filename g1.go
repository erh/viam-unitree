package main

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

var g1Model = resource.NewModel("erh", "viam-unitree", "g1")

type G1Config struct {
	NetworkInterface string `json:"network_interface"`
}

func (c *G1Config) Validate(path string) ([]string, error) {
	return nil, nil
}

func init() {
	resource.RegisterComponent(generic.API, g1Model, resource.Registration[resource.Resource, *G1Config]{
		Constructor: newG1,
	})
}

type g1 struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	loco   *LocoClient
}

func newG1(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*G1Config](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}

	logger.Infof("Initializing G1 with network interface: %s", networkInterface)

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	loco, err := NewLocoClient()
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("loco client: %w", err)
	}

	logger.Info("G1 initialized successfully")

	return &g1{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		loco:   loco,
	}, nil
}

// readyToMove runs the post-boot sequence to get the G1 into a state where
// it can accept locomotion commands:
//
//	Damp (FSM 1) → Squat2StandUp (FSM 706) → settle → ContinuousGait(true)
//
// Damp puts the joints into a known damped state so the Squat2StandUp
// transition is accepted. Squat2StandUp (FSM 706) is the current-firmware
// transition that replaces the legacy StandUp (FSM 4) — the legacy FSM 4 is
// silently ignored on recent firmware (RPC returns rc=0 but FSM does not
// change), which previously left the robot unresponsive to Move commands.
//
// After the robot is standing, we must switch into "walk mode" by setting
// BalanceMode 1 (ContinuousGait enabled). The default post-StandUp balance
// mode is 0 (static balance): the robot stands firm but ignores SetVelocity
// entirely. Without this step, ready_to_move appears to succeed (robot
// stands) but Move commands silently do nothing — matching the symptom the
// user reported.
func (g *g1) readyToMove(ctx context.Context) error {
	g.logger.Info("readyToMove: issuing damp")
	if _, err := g.loco.Damp(); err != nil {
		return fmt.Errorf("damp: %w", err)
	}

	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(500 * time.Millisecond):
	}

	g.logger.Info("readyToMove: issuing squat_to_stand_up (FSM 706)")
	if _, err := g.loco.Squat2StandUp(); err != nil {
		return fmt.Errorf("squat_to_stand_up: %w", err)
	}

	// Give the robot time to reach the standing pose before switching into
	// walk mode. Issuing ContinuousGait mid-transition can be rejected.
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(5 * time.Second):
	}

	g.logger.Info("readyToMove: enabling continuous gait (walk mode)")
	if _, err := g.loco.ContinuousGait(true); err != nil {
		return fmt.Errorf("continuous_gait: %w", err)
	}

	// Short settle so the first Move() call doesn't race the mode switch.
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(500 * time.Millisecond):
	}

	g.logger.Info("readyToMove: complete")
	return nil
}

// armGestures maps DoCommand strings to the LocoClient arm-task wrappers.
// These are pre-recorded G1 arm motions exposed by the sport service via
// SetArmTask. They run to completion on the robot side.
var armGestures = map[string]int{
	"wave_hand":     ArmTaskWaveHand,
	"turn_wave":     ArmTaskTurnWave,
	"release_arm":   ArmTaskReleaseArm,
	"shake_hand":    ArmTaskShakeHand,
	"high_five":     ArmTaskHighFive,
	"hug":           ArmTaskHug,
	"heart":         ArmTaskHeart,
	"refuse":        ArmTaskRefuse,
	"right_kiss":    ArmTaskRightKiss,
	"left_kiss":     ArmTaskLeftKiss,
	"two_hand_kiss": ArmTaskTwoHandKiss,
	"hands_up":      ArmTaskHandsUp,
	"clap":          ArmTaskClap,
	"face_wave":     ArmTaskFaceWave,
	"high_wave":     ArmTaskHighWave,
}

func (g *g1) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cmdStr, ok := cmd["command"].(string)
	if !ok {
		return map[string]interface{}{"error": "missing 'command' field"}, nil
	}

	// Built-in arm gestures triggered via SetArmTask.
	if taskID, isGesture := armGestures[cmdStr]; isGesture {
		if err := g.loco.SetArmTask(taskID); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		return map[string]interface{}{"rc": 0.0}, nil
	}

	switch cmdStr {
	case "ready_to_move":
		if err := g.readyToMove(ctx); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		return map[string]interface{}{"rc": 0.0}, nil
	case "set_arm_task":
		// Generic passthrough for any task ID, including IDs not in the
		// armGestures map. Accepts task_id as a number.
		raw, ok := cmd["task_id"]
		if !ok {
			return map[string]interface{}{"rc": -1.0, "error": "missing 'task_id'"}, nil
		}
		taskID, err := numericToInt(raw)
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		if err := g.loco.SetArmTask(taskID); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		return map[string]interface{}{"rc": 0.0}, nil
	default:
		return map[string]interface{}{"error": "unknown command: " + cmdStr}, nil
	}
}

// numericToInt accepts the JSON numeric flavors that map[string]interface{}
// can hold (float64 from Viam DoCommand decoding, plus the integer types).
func numericToInt(v interface{}) (int, error) {
	switch x := v.(type) {
	case float64:
		return int(x), nil
	case int:
		return x, nil
	case int64:
		return int(x), nil
	case int32:
		return int(x), nil
	default:
		return 0, fmt.Errorf("expected numeric, got %T", v)
	}
}

func (g *g1) Close(ctx context.Context) error {
	if g.loco != nil {
		g.loco.Close()
	}
	ShutdownDDS()
	return nil
}
