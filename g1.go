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

// readyToMove runs the post-boot sequence to get the G1 into a state
// that accepts Move commands. Sequence:
//   Damp (1) → StandUp (4) → SetBalanceMode(1) → Run (802)
//
// This is intentionally minimal — just 4 RPC calls with sleeps between.
// Issuing many rapid RPCs (logFsm reads, transitionFsm pre/post checks)
// creates enough DDS waitset churn on the shared participant to corrupt
// internal CycloneDDS state and break the videohub (camera) reader.
// The manual DoCommand equivalents of these 4 calls were verified to
// leave both walking and camera functional.
func (g *g1) readyToMove(ctx context.Context) error {
	g.logger.Info("readyToMove: Damp")
	if err := g.loco.SetFsmID(FsmDamp); err != nil {
		g.logger.Warnf("readyToMove: Damp RPC error (may already be in state): %v", err)
	}
	if err := sleepCtx(ctx, 1*time.Second); err != nil {
		return err
	}

	g.logger.Info("readyToMove: StandUp")
	if err := g.loco.SetFsmID(FsmStandUp); err != nil {
		return fmt.Errorf("stand_up: %w", err)
	}
	if err := sleepCtx(ctx, 5*time.Second); err != nil {
		return err
	}

	g.logger.Info("readyToMove: SetBalanceMode(1)")
	if err := g.loco.SetBalanceMode(1); err != nil {
		return fmt.Errorf("set_balance_mode: %w", err)
	}
	if err := sleepCtx(ctx, 500*time.Millisecond); err != nil {
		return err
	}

	g.logger.Info("readyToMove: Run")
	if err := g.loco.SetFsmID(FsmRun); err != nil {
		return fmt.Errorf("run: %w", err)
	}
	if err := sleepCtx(ctx, 1*time.Second); err != nil {
		return err
	}

	g.logger.Info("readyToMove: complete")
	return nil
}

// transitionFsm issues an FSM-setting command and verifies the firmware
// actually moved to expectedID. Returns nil if the robot is already in
// the target state (idempotent). Returns an error if the RPC fails OR
// if the FSM doesn't end up at expectedID after the settle wait (silent
// rejection).
func (g *g1) transitionFsm(ctx context.Context, name string, expectedID int, call func() (int, error), settle time.Duration) error {
	current, err := g.loco.GetFsmID()
	if err != nil {
		return fmt.Errorf("%s: read FSM before: %w", name, err)
	}
	if current == expectedID {
		g.logger.Infof("readyToMove: %s: already in FSM=%d, skipping", name, expectedID)
		return nil
	}

	g.logger.Infof("readyToMove: issuing %s (FSM %d → %d)", name, current, expectedID)
	if _, err := call(); err != nil {
		return fmt.Errorf("%s RPC: %w", name, err)
	}
	if err := sleepCtx(ctx, settle); err != nil {
		return err
	}
	g.logFsm(fmt.Sprintf("readyToMove: after %s", name))

	after, err := g.loco.GetFsmID()
	if err != nil {
		return fmt.Errorf("%s: read FSM after: %w", name, err)
	}
	if after != expectedID {
		return fmt.Errorf("%s: FSM stayed at %d (expected %d) — silent rejection", name, after, expectedID)
	}
	return nil
}

// logFsm queries and logs the current FSM id, FSM mode, and balance
// mode. Any individual getter that fails is reported as "?" rather than
// suppressing the whole line — GetBalanceMode in particular returns
// status=7301 on some firmwares but the FSM getters still work.
func (g *g1) logFsm(label string) {
	fmtVal := func(v int, err error) string {
		if err != nil {
			return "?(" + err.Error() + ")"
		}
		return fmt.Sprintf("%d", v)
	}
	fsmID, idErr := g.loco.GetFsmID()
	fsmMode, modeErr := g.loco.GetFsmMode()
	balMode, balErr := g.loco.GetBalanceMode()
	g.logger.Infof("%s: fsm_id=%s fsm_mode=%s balance_mode=%s",
		label, fmtVal(fsmID, idErr), fmtVal(fsmMode, modeErr), fmtVal(balMode, balErr))
}

func sleepCtx(ctx context.Context, d time.Duration) error {
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(d):
		return nil
	}
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
	case "set_fsm_id":
		// Set an arbitrary FSM ID. Returns fsm_id before/after so you
		// can see if it landed. Usage: {"command":"set_fsm_id","id":802}
		raw, ok := cmd["id"]
		if !ok {
			return map[string]interface{}{"rc": -1.0, "error": "missing 'id'"}, nil
		}
		id, err := numericToInt(raw)
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		before, _ := g.loco.GetFsmID()
		rpcErr := g.loco.SetFsmID(id)
		time.Sleep(1 * time.Second)
		after, _ := g.loco.GetFsmID()
		result := map[string]interface{}{
			"rc":           0.0,
			"id_requested": id,
			"fsm_before":   before,
			"fsm_after":    after,
			"changed":      before != after,
		}
		if rpcErr != nil {
			result["rpc_error"] = rpcErr.Error()
		}
		g.logger.Infof("set_fsm_id(%d): %d → %d (rpc_err=%v)", id, before, after, rpcErr)
		return result, nil
	case "get_modes":
		// Read every state/mode the sport service exposes. Use this
		// while the robot is in a known-good state (e.g. walking via
		// remote) to capture the exact values we need to reproduce.
		result := map[string]interface{}{"rc": 0.0}
		readInt := func(key string, fn func() (int, error)) {
			v, err := fn()
			if err != nil {
				result[key+"_err"] = err.Error()
			} else {
				result[key] = v
			}
		}
		readFloat := func(key string, fn func() (float64, error)) {
			v, err := fn()
			if err != nil {
				result[key+"_err"] = err.Error()
			} else {
				result[key] = v
			}
		}
		readInt("fsm_id", g.loco.GetFsmID)
		readInt("fsm_mode", g.loco.GetFsmMode)
		readInt("balance_mode", g.loco.GetBalanceMode)
		readFloat("swing_height", g.loco.GetSwingHeight)
		readFloat("stand_height", g.loco.GetStandHeight)
		g.logger.Infof("get_modes: %+v", result)
		return result, nil
	case "set_speed_mode":
		// Set speed mode (API 7107). Usage: {"command":"set_speed_mode","mode":<n>}
		raw, ok := cmd["mode"]
		if !ok {
			return map[string]interface{}{"rc": -1.0, "error": "missing 'mode'"}, nil
		}
		mode, err := numericToInt(raw)
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		if err := g.loco.SetSpeedMode(mode); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		return map[string]interface{}{"rc": 0.0}, nil
	case "set_balance_mode":
		// Experiment helper: calls SetBalanceMode with the given mode
		// and logs fsm_mode before/after so you can see what the
		// firmware does. Usage: {"command":"set_balance_mode","mode":1}
		raw, ok := cmd["mode"]
		if !ok {
			return map[string]interface{}{"rc": -1.0, "error": "missing 'mode'"}, nil
		}
		mode, err := numericToInt(raw)
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		before, _ := g.loco.GetFsmMode()
		rpcErr := g.loco.SetBalanceMode(mode)
		time.Sleep(500 * time.Millisecond)
		after, _ := g.loco.GetFsmMode()
		result := map[string]interface{}{
			"rc":                 0.0,
			"mode_requested":     mode,
			"fsm_mode_before":    before,
			"fsm_mode_after":     after,
		}
		if rpcErr != nil {
			result["rpc_error"] = rpcErr.Error()
		}
		g.logger.Infof("set_balance_mode(%d): fsm_mode %d → %d (rpc_err=%v)", mode, before, after, rpcErr)
		return result, nil
	case "try_fsm":
		// Diagnostic: iterate through every FSM ID that Unitree firmware
		// has ever published, set each, and report which ones actually
		// move the FSM. Intended to be run once from whatever state the
		// robot is in, to figure out which transitions the current
		// firmware honors. Does NOT pause arm_sdk — caller should ensure
		// arm_sdk is dormant first.
		candidates := []int{
			FsmZeroTorque, FsmDamp, FsmSquat, FsmSit, FsmStandUp,
			5, 6, 7, 8, // speculative mid-range IDs some firmwares use
			FsmStart,
			500, // legacy Start
			FsmLie2StandUp, FsmSquat2StandUp,
			701, 703, 704, 705, 707, // neighbors of 702/706
		}
		results := []map[string]interface{}{}
		for _, id := range candidates {
			before, err := g.loco.GetFsmID()
			if err != nil {
				results = append(results, map[string]interface{}{"fsm_id": id, "error": "read_before: " + err.Error()})
				continue
			}
			rpcErr := g.loco.SetFsmID(id)
			time.Sleep(1 * time.Second)
			after, _ := g.loco.GetFsmID()
			entry := map[string]interface{}{
				"fsm_id":    id,
				"before":    before,
				"after":     after,
				"changed":   before != after,
				"rpc_error": "",
			}
			if rpcErr != nil {
				entry["rpc_error"] = rpcErr.Error()
			}
			results = append(results, entry)
			g.logger.Infof("try_fsm: id=%d before=%d after=%d changed=%v rpc_error=%v", id, before, after, before != after, rpcErr)
		}
		return map[string]interface{}{"results": results}, nil
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
