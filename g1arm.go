package main

import (
	"context"
	_ "embed"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/referenceframe/urdf"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	g1LeftArmModel  = resource.NewModel("erh", "viam-unitree", "g1-left-arm")
	g1RightArmModel = resource.NewModel("erh", "viam-unitree", "g1-right-arm")
)

//go:embed g1_left_arm_model.json
var g1LeftArmKinematics []byte

//go:embed g1_right_arm_model.json
var g1RightArmKinematics []byte

// G1ArmConfig configures a g1-(left|right)-arm.
//
// model_path is optional. If supplied, it is read from disk and replaces the
// embedded best-effort kinematic model. Use this if your G1 has different
// link lengths than the defaults baked into this module.
type G1ArmConfig struct {
	NetworkInterface string `json:"network_interface"`

	// ModelPath, when set, overrides the embedded kinematics file (.json or .urdf).
	ModelPath string `json:"model_path,omitempty"`

	// Kp / Kd are the per-joint position/velocity gains used for arm_sdk
	// motor commands. Sensible defaults are chosen if omitted; tune as
	// needed for your robot. Length must equal G1ArmDoF (7) if specified.
	Kp []float64 `json:"kp,omitempty"`
	Kd []float64 `json:"kd,omitempty"`
}

func (c *G1ArmConfig) Validate(path string) ([]string, error) {
	if c.Kp != nil && len(c.Kp) != G1ArmDoF {
		return nil, errors.Errorf("kp must have %d entries, got %d", G1ArmDoF, len(c.Kp))
	}
	if c.Kd != nil && len(c.Kd) != G1ArmDoF {
		return nil, errors.Errorf("kd must have %d entries, got %d", G1ArmDoF, len(c.Kd))
	}
	return nil, nil
}

func init() {
	resource.RegisterComponent(arm.API, g1LeftArmModel, resource.Registration[arm.Arm, *G1ArmConfig]{
		Constructor: func(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (arm.Arm, error) {
			return newG1Arm(ctx, conf, logger, sideLeft)
		},
	})
	resource.RegisterComponent(arm.API, g1RightArmModel, resource.Registration[arm.Arm, *G1ArmConfig]{
		Constructor: func(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (arm.Arm, error) {
			return newG1Arm(ctx, conf, logger, sideRight)
		},
	})
}

type armSide int

const (
	sideLeft armSide = iota
	sideRight
)

// Default position/velocity gains. The G1 arm motors are relatively low
// gear-ratio direct drive so values in this range are a reasonable start.
// Tune via the Kp/Kd config keys if needed.
var (
	defaultArmKp = []float32{60, 60, 60, 60, 30, 30, 30}
	defaultArmKd = []float32{1.5, 1.5, 1.5, 1.5, 1.0, 1.0, 1.0}
)

type g1Arm struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	sdk    *armSDK

	side    armSide
	indices []int

	model referenceframe.Model

	kp []float32
	kd []float32

	// Last commanded joint positions (radians). Used as a fallback when no
	// LowState has been received yet, and as the synchronous setpoint for
	// IsMoving / Geometries.
	mu          sync.RWMutex
	commanded   []float64
	moving      atomic.Bool
	lastCmdTime time.Time
}

func newG1Arm(ctx context.Context, conf resource.Config, logger logging.Logger, side armSide) (arm.Arm, error) {
	cfg, err := resource.NativeConfig[*G1ArmConfig](conf)
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

	sdk, err := getArmSDK()
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("arm_sdk client: %w", err)
	}

	var (
		model    referenceframe.Model
		indices  []int
		modelRaw []byte
	)
	switch side {
	case sideLeft:
		modelRaw = g1LeftArmKinematics
		indices = LeftArmJointIndices
	case sideRight:
		modelRaw = g1RightArmKinematics
		indices = RightArmJointIndices
	}

	if cfg.ModelPath != "" {
		model, err = loadArmModel(cfg.ModelPath, conf.Name)
	} else {
		model, err = referenceframe.UnmarshalModelJSON(modelRaw, conf.Name)
	}
	if err != nil {
		sdk.release()
		ShutdownDDS()
		return nil, fmt.Errorf("load arm model: %w", err)
	}
	if got := len(model.DoF()); got != G1ArmDoF {
		sdk.release()
		ShutdownDDS()
		return nil, fmt.Errorf("arm model must have %d DoF, got %d", G1ArmDoF, got)
	}

	kp := append([]float32(nil), defaultArmKp...)
	kd := append([]float32(nil), defaultArmKd...)
	for i, v := range cfg.Kp {
		kp[i] = float32(v)
	}
	for i, v := range cfg.Kd {
		kd[i] = float32(v)
	}

	a := &g1Arm{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		sdk:       sdk,
		side:      side,
		indices:   indices,
		model:     model,
		kp:        kp,
		kd:        kd,
		commanded: make([]float64, G1ArmDoF),
	}

	// Take ownership of the arms (weight=1 routes arm_sdk commands through).
	sdk.setWeight(1.0)
	logger.Infof("g1Arm (%s) initialized with %d DoF", a.sideName(), len(model.DoF()))
	return a, nil
}

func (a *g1Arm) sideName() string {
	if a.side == sideLeft {
		return "left"
	}
	return "right"
}

func loadArmModel(path, name string) (referenceframe.Model, error) {
	if len(path) > 5 && path[len(path)-5:] == ".urdf" {
		return urdf.ParseModelXMLFile(path, name)
	}
	return referenceframe.ParseModelJSONFile(path, name)
}

func (a *g1Arm) ModelFrame() referenceframe.Model {
	return a.model
}

func (a *g1Arm) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	// Prefer measured positions from rt/lowstate; fall back to the most
	// recent setpoint if nothing has been received yet.
	if pos, ok := a.sdk.jointPositions(a.indices); ok {
		out := make([]referenceframe.Input, len(pos))
		for i, p := range pos {
			out[i] = referenceframe.Input{Value: float64(p)}
		}
		return out, nil
	}

	a.mu.RLock()
	defer a.mu.RUnlock()
	out := make([]referenceframe.Input, len(a.commanded))
	for i, p := range a.commanded {
		out[i] = referenceframe.Input{Value: p}
	}
	return out, nil
}

func (a *g1Arm) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	joints, err := a.JointPositions(ctx, extra)
	if err != nil {
		return nil, err
	}
	return referenceframe.ComputeOOBPosition(a.model, joints)
}

func (a *g1Arm) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	current, err := a.JointPositions(ctx, extra)
	if err != nil {
		return err
	}

	plan, err := motionplan.PlanFrameMotion(ctx, a.logger, pose, a.model, current, nil, nil)
	if err != nil {
		return err
	}
	return a.MoveThroughJointPositions(ctx, plan, nil, extra)
}

func (a *g1Arm) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	if err := arm.CheckDesiredJointPositions(ctx, a, positions); err != nil {
		return err
	}
	return a.commandPositions(ctx, positions)
}

func (a *g1Arm) MoveThroughJointPositions(
	ctx context.Context,
	positions [][]referenceframe.Input,
	_ *arm.MoveOptions,
	extra map[string]interface{},
) error {
	for _, step := range positions {
		if err := a.MoveToJointPositions(ctx, step, extra); err != nil {
			return err
		}
		// Hold each waypoint briefly so the arm has a chance to settle
		// before we set the next one.
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(100 * time.Millisecond):
		}
	}
	return nil
}

// commandPositions writes new setpoints into the shared LowCmd snapshot. The
// publisher loop on armSDK handles the actual DDS write at its own cadence.
func (a *g1Arm) commandPositions(ctx context.Context, positions []referenceframe.Input) error {
	if len(positions) != len(a.indices) {
		return fmt.Errorf("expected %d joint positions, got %d", len(a.indices), len(positions))
	}

	q := make([]float32, len(positions))
	for i, p := range positions {
		q[i] = float32(p.Value)
	}

	a.mu.Lock()
	for i, p := range positions {
		a.commanded[i] = p.Value
	}
	a.lastCmdTime = time.Now()
	a.mu.Unlock()

	a.moving.Store(true)
	if err := a.sdk.setArmCommand(a.indices, q, a.kp, a.kd); err != nil {
		a.moving.Store(false)
		return err
	}
	// Approximate "done moving" - the publisher keeps reasserting the
	// setpoint, but we don't have a true motion-complete signal so fall
	// back to a short delay. Callers that need precise blocking can poll
	// JointPositions themselves.
	go func() {
		select {
		case <-ctx.Done():
		case <-time.After(500 * time.Millisecond):
		}
		a.moving.Store(false)
	}()
	return nil
}

func (a *g1Arm) Stop(ctx context.Context, extra map[string]interface{}) error {
	// "Stop" for arm_sdk means latch the current commanded position. Since
	// we already publish position setpoints continuously, holding the last
	// commanded position is the right behavior; just clear the moving flag.
	a.moving.Store(false)
	return nil
}

func (a *g1Arm) IsMoving(ctx context.Context) (bool, error) {
	return a.moving.Load(), nil
}

func (a *g1Arm) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return a.JointPositions(ctx, nil)
}

func (a *g1Arm) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return a.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (a *g1Arm) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := a.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := a.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (a *g1Arm) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cmdStr, _ := cmd["command"].(string)
	switch cmdStr {
	case "release":
		// Surrender control back to sport mode (weight=0).
		a.sdk.setWeight(0)
		return map[string]interface{}{"rc": 0.0}, nil
	case "engage":
		a.sdk.setWeight(1)
		return map[string]interface{}{"rc": 0.0}, nil
	case "set_weight":
		w, err := numericToFloat(cmd["weight"])
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		a.sdk.setWeight(float32(w))
		return map[string]interface{}{"rc": 0.0}, nil
	case "":
		return map[string]interface{}{}, nil
	default:
		return map[string]interface{}{"error": "unknown command: " + cmdStr}, nil
	}
}

func (a *g1Arm) Close(ctx context.Context) error {
	a.sdk.release()
	ShutdownDDS()
	return nil
}

func numericToFloat(v interface{}) (float64, error) {
	switch x := v.(type) {
	case float64:
		return x, nil
	case float32:
		return float64(x), nil
	case int:
		return float64(x), nil
	case int64:
		return float64(x), nil
	default:
		return 0, fmt.Errorf("expected numeric, got %T", v)
	}
}
