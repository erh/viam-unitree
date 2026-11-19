package main

import (
	"context"
	"fmt"
	"slices"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/grpc"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	g1LeftArmModel  = resource.NewModel("erh", "viam-unitree", "g1-left-arm")
	g1RightArmModel = resource.NewModel("erh", "viam-unitree", "g1-right-arm")
	g1BodyModel     = resource.NewModel("erh", "viam-unitree", "g1-body")
)

// G1ArmConfig configures a g1-(left|right)-arm.
//
// model_path is optional. If supplied, it is read from disk and replaces the
// kinematics built from the embedded G1 URDF. Joint names must be G1 joint
// names (e.g. left_elbow or left_elbow_joint) so they map to motors.
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

func (c *G1ArmConfig) Validate(path string) ([]string, []string, error) {
	if c.Kp != nil && len(c.Kp) != G1ArmDoF {
		return nil, nil, errors.Errorf("kp must have %d entries, got %d", G1ArmDoF, len(c.Kp))
	}
	if c.Kd != nil && len(c.Kd) != G1ArmDoF {
		return nil, nil, errors.Errorf("kd must have %d entries, got %d", G1ArmDoF, len(c.Kd))
	}
	return nil, nil, nil
}

// G1BodyConfig configures a g1-body. It has G1ArmConfig's fields, but the body is read-only, so
// it takes no gains.
type G1BodyConfig G1ArmConfig

func (c *G1BodyConfig) Validate(path string) ([]string, []string, error) {
	if c.Kp != nil || c.Kd != nil {
		return nil, nil, errors.New("g1-body is read-only; kp and kd apply only to the arms")
	}
	return nil, nil, nil
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
	resource.RegisterComponent(arm.API, g1BodyModel, resource.Registration[arm.Arm, *G1BodyConfig]{
		Constructor: func(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (arm.Arm, error) {
			return newG1Arm(ctx, conf, logger, sideBody)
		},
	})
}

type armSide int

const (
	sideLeft armSide = iota
	sideRight
	// sideBody is pelvis, legs and waist: read-only, since arm_sdk can't drive the legs.
	sideBody
)

func (s armSide) String() string {
	switch s {
	case sideLeft:
		return "left arm"
	case sideRight:
		return "right arm"
	default:
		return "body"
	}
}

// motors is the rt/lowstate motor index range [lo, hi) that the side's model must cover.
func (s armSide) motors() (lo, hi int) {
	switch s {
	case sideLeft:
		return G1JointLeftShoulderPitch, G1JointLeftWristYaw + 1
	case sideRight:
		return G1JointRightShoulderPitch, G1JointRightWristYaw + 1
	default:
		return G1JointLeftHipPitch, G1JointWaistPitch + 1
	}
}

// checkJoints reports an error unless indices, a model's motor indices from g1JointIndices, hold
// each of the side's motors exactly once, in any order.
func (s armSide) checkJoints(indices []int) error {
	lo, hi := s.motors()
	seen := make([]bool, hi-lo)
	for _, idx := range indices {
		if idx < lo || idx >= hi {
			return fmt.Errorf("the %s model has joint %s, which is not a %s joint", s, g1MotorJoints[idx], s)
		}
		if seen[idx-lo] {
			return fmt.Errorf("the %s model has joint %s twice", s, g1MotorJoints[idx])
		}
		seen[idx-lo] = true
	}
	if i := slices.Index(seen, false); i >= 0 {
		return fmt.Errorf("the %s model must have %d DoF, got %d: it is missing %s", s, hi-lo, len(indices), g1MotorJoints[lo+i])
	}
	return nil
}

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
	var cfg *G1ArmConfig
	var err error
	if side == sideBody {
		var bc *G1BodyConfig
		bc, err = resource.NativeConfig[*G1BodyConfig](conf)
		cfg = (*G1ArmConfig)(bc)
	} else {
		cfg, err = resource.NativeConfig[*G1ArmConfig](conf)
	}
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

	sdk, err := getArmSDK(logger)
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("arm_sdk client: %w", err)
	}

	var model referenceframe.Model
	switch {
	case cfg.ModelPath != "":
		model, err = referenceframe.KinematicModelFromFile(cfg.ModelPath, conf.Name)
	case side == sideLeft:
		model, err = g1LeftArmKinematics(conf.Name)
	case side == sideRight:
		model, err = g1RightArmKinematics(conf.Name)
	default:
		model, err = g1BodyKinematics(conf.Name)
	}
	var indices []int
	if err == nil {
		indices, err = g1JointIndices(model)
	}
	if err == nil {
		err = side.checkJoints(indices)
	}
	if err != nil {
		sdk.release()
		ShutdownDDS()
		return nil, fmt.Errorf("load arm model: %w", err)
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
		commanded: make([]float64, len(indices)),
	}

	// arm_sdk starts released (weight=0, no DDS writer), leaving the robot
	// to the sport controller. The first arm move engages it (or use the
	// "engage" DoCommand); gestures, ready_to_move and FSM changes release
	// it again, as does the "release" DoCommand.
	logger.Infof("g1 %s initialized with %d DoF", side, len(model.DoF()))
	return a, nil
}

func (a *g1Arm) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return a.model, nil
}

func (a *g1Arm) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	// Prefer measured positions from rt/lowstate; fall back to the most
	// recent setpoint if nothing has been received yet.
	if pos, ok := a.sdk.jointPositions(a.indices); ok {
		out := make([]referenceframe.Input, len(pos))
		for i, p := range pos {
			out[i] = float64(p)
		}
		return out, nil
	}
	if a.side == sideBody {
		// The body is never commanded, so there is no setpoint to fall back to.
		return nil, errors.New("no rt/lowstate received yet")
	}

	a.mu.RLock()
	defer a.mu.RUnlock()
	return append([]referenceframe.Input(nil), a.commanded...), nil
}

func (a *g1Arm) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	joints, err := a.JointPositions(ctx, extra)
	if err != nil {
		return nil, err
	}
	return a.model.Transform(joints)
}

func (a *g1Arm) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	return fmt.Errorf("MoveToPosition not supported")
}

func (a *g1Arm) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
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

// commandPositions checks and writes new setpoints into the shared LowCmd
// snapshot. The publisher loop on armSDK handles the actual DDS write at its
// own cadence. It is the only path to motor commands, so the body is refused
// here, before anything else runs.
func (a *g1Arm) commandPositions(ctx context.Context, positions []referenceframe.Input) error {
	if a.side == sideBody {
		return errors.New("g1-body is read-only; move the arms with g1-left-arm / g1-right-arm")
	}
	// Check the length first: CheckDesiredJointPositions indexes the limits by input and panics
	// on extra inputs.
	if len(positions) != len(a.indices) {
		return fmt.Errorf("expected %d joint positions, got %d", len(a.indices), len(positions))
	}
	if err := arm.CheckDesiredJointPositions(ctx, a, positions); err != nil {
		return err
	}

	q := make([]float32, len(positions))
	for i, p := range positions {
		q[i] = float32(p)
	}

	a.mu.Lock()
	copy(a.commanded, positions)
	a.lastCmdTime = time.Now()
	a.mu.Unlock()

	a.moving.Store(true)
	// Auto-engage: take arm_sdk ownership (weight→1) on the first move so
	// jogging from the UI works without a manual "engage" DoCommand. It holds
	// the current pose while engaging, waits out a release in progress, and
	// is a no-op once engaged.
	if err := a.sdk.ensureEngaged(); err != nil {
		a.moving.Store(false)
		return err
	}
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

// MoveThroughJointPositionsStreamed executes a streamed trajectory by applying
// each waypoint in order. We have no time model, so per-point Time/Constraints
// are ignored; we command each waypoint and acknowledge once per batch.
func (a *g1Arm) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	_ map[string]interface{},
) error {
	for batch := range batches {
		for _, p := range batch {
			if err := a.MoveToJointPositions(ctx, p.Positions, nil); err != nil {
				return err
			}
		}
		select {
		case responses <- arm.Response{}:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	return nil
}

// Get3DModels returns the visual meshes the arm's kinematic model carries,
// keyed by link. The embedded G1 arm models ship none, so this is empty.
func (a *g1Arm) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	models := referenceframe.VisualMeshes(a.model)
	if models == nil {
		models = map[string]*commonpb.Mesh{}
	}
	return models, nil
}

func (a *g1Arm) Properties(ctx context.Context, extra map[string]interface{}) (arm.Properties, error) {
	return arm.Properties{
		SupportManualMode:        false,
		SupportCartesianCommands: false,
	}, nil
}

func (a *g1Arm) SetManualMode(ctx context.Context, manualMode bool, enabledFor time.Duration, extra map[string]interface{}) error {
	return grpc.UnimplementedError
}

func (a *g1Arm) ManualMode(ctx context.Context, extra map[string]interface{}) (bool, error) {
	return false, grpc.UnimplementedError
}

func (a *g1Arm) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cmdStr, _ := cmd["command"].(string)
	if a.side == sideBody && cmdStr != "" {
		// arm_sdk weight is shared with the arms; don't let the read-only body take them over.
		return map[string]interface{}{"error": "g1-body has no commands; use an arm component"}, nil
	}
	switch cmdStr {
	case "release":
		// Fade out, then close the writer, handing the upper body back to sport.
		if err := a.sdk.disengage(); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		return map[string]interface{}{"rc": 0.0}, nil
	case "engage":
		// Gradually ramp weight up for smooth arm_sdk takeover, holding the
		// current pose.
		if err := a.sdk.rampWeight(1, 1*time.Second); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		return map[string]interface{}{"rc": 0.0}, nil
	case "set_weight":
		w, err := numericToFloat(cmd["weight"])
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		if err := a.sdk.setWeight(float32(w)); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		return map[string]interface{}{"rc": 0.0}, nil
	case "gravity_compensation":
		// Reads, or with "scale" sets, the feedforward that holds the waist and
		// arms against gravity (shared by both arms): 0 off, 1 the full model.
		// Returns the torque each upper-body joint gets at the current pose.
		if raw, ok := cmd["scale"]; ok {
			scale, err := numericToFloat(raw)
			if err != nil {
				return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
			}
			if err := a.sdk.setGravityScale(scale); err != nil {
				return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
			}
		}
		scale, torques, err := a.sdk.gravityStatus()
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		torqueNm := map[string]interface{}{}
		for name, t := range torques {
			torqueNm[name] = t
		}
		return map[string]interface{}{"rc": 0.0, "scale": scale, "torque_nm": torqueNm}, nil
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
