package main

import (
	"context"
	"errors"
	"fmt"
	"sort"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// g1-hand drives one BrainCo Revo2 hand on the G1 as a gripper. It talks
// Modbus RTU to the hand directly over the Jetson's USB-to-RS485 adapter, so
// no Unitree hand service needs to run.
var g1HandModel = resource.NewModel("erh", "viam-unitree", "g1-hand")

// Named poses, in percent closed (0 = open, 100 = closed), finger order
// thumb, thumb_aux, index, middle, ring, pinky. "fist" keeps the thumb half
// closed so it folds over the fingers instead of into them, matching
// BrainCo's own close pose.
var g1HandPoses = map[string][brainCoNumFingers]int{
	"open":      {0, 0, 0, 0, 0, 0},
	"fist":      {50, 50, 100, 100, 100, 100},
	"point":     {50, 50, 0, 100, 100, 100},
	"peace":     {50, 50, 0, 0, 100, 100},
	"thumbs_up": {0, 0, 100, 100, 100, 100},
	"pinch":     {60, 70, 60, 0, 0, 0},
}

const (
	g1HandMoveTimeout = 3 * time.Second
	g1HandPoll        = 50 * time.Millisecond
	// A finger that stops this far (0–1000 units) short of its grab target is
	// treated as blocked by an object.
	g1HandBlockedMargin = 150
)

type g1HandConfig struct {
	// Side is "left" or "right". It picks the default Modbus slave id and is
	// checked against the hand's SKU at startup.
	Side string `json:"side"`
	// SerialPath is the hand's RS485 port, e.g.
	// /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_<serial>-if01-port0.
	SerialPath string `json:"serial_path"`
	// SlaveID overrides the Modbus id (default 126 left, 127 right).
	SlaveID int `json:"slave_id,omitempty"`
	// GrabPose names the pose Grab closes to (default "fist").
	GrabPose string `json:"grab_pose,omitempty"`
}

func (c *g1HandConfig) Validate(path string) ([]string, []string, error) {
	if c.Side != "left" && c.Side != "right" {
		return nil, nil, fmt.Errorf(`%s: "side" must be "left" or "right"`, path)
	}
	if c.SerialPath == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "serial_path")
	}
	if c.SlaveID < 0 || c.SlaveID > 247 {
		return nil, nil, fmt.Errorf("%s: slave_id must be 1..247", path)
	}
	if c.GrabPose != "" {
		if _, ok := g1HandPoses[c.GrabPose]; !ok {
			return nil, nil, fmt.Errorf("%s: unknown grab_pose %q", path, c.GrabPose)
		}
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(gripper.API, g1HandModel,
		resource.Registration[gripper.Gripper, *g1HandConfig]{
			Constructor: newG1Hand,
		})
}

type g1Hand struct {
	resource.Named
	resource.AlwaysRebuild

	logger   logging.Logger
	hand     *BrainCoHand
	grabPose [brainCoNumFingers]int
	model    referenceframe.Model
}

func newG1Hand(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*g1HandConfig](conf)
	if err != nil {
		return nil, err
	}

	slaveID := cfg.SlaveID
	if slaveID == 0 {
		slaveID = 127
		if cfg.Side == "left" {
			slaveID = 126
		}
	}
	grabPose := g1HandPoses["fist"]
	if cfg.GrabPose != "" {
		grabPose = g1HandPoses[cfg.GrabPose]
	}

	hand, err := OpenBrainCoHand(cfg.SerialPath, byte(slaveID))
	if err != nil {
		return nil, err
	}

	// Make sure this port has the hand we were told it has, so a swapped
	// cable can't send left-hand commands to the right hand.
	sku, err := hand.SKU()
	if err != nil {
		hand.Close()
		return nil, fmt.Errorf("no BrainCo hand answered on %s as slave %d: %w", cfg.SerialPath, slaveID, err)
	}
	skuSide := map[int]string{1: "right", 3: "right", 2: "left", 4: "left"}[sku]
	if skuSide != cfg.Side {
		hand.Close()
		return nil, fmt.Errorf("hand on %s reports SKU %d (%s), but config says side %q", cfg.SerialPath, sku, skuSide, cfg.Side)
	}
	if fw, err := hand.Firmware(); err == nil {
		logger.Infof("g1-hand: %s BrainCo hand on %s (slave %d, SKU %d, firmware %s)", cfg.Side, cfg.SerialPath, slaveID, sku, fw)
	}

	model, err := gripper.MakeModel(conf.ResourceName().ShortName(), nil)
	if err != nil {
		hand.Close()
		return nil, err
	}

	return &g1Hand{
		Named:    conf.ResourceName().AsNamed(),
		logger:   logger,
		hand:     hand,
		grabPose: grabPose,
		model:    model,
	}, nil
}

// moveTo commands targets (percent closed) and waits until the fingers stop
// moving or the timeout passes. It returns the last status read.
func (h *g1Hand) moveTo(ctx context.Context, targets [brainCoNumFingers]int) (BrainCoStatus, error) {
	if err := h.hand.SetTargetsPercent(targets); err != nil {
		return BrainCoStatus{}, err
	}
	deadline := time.Now().Add(g1HandMoveTimeout)
	var last BrainCoStatus
	stillCount := 0
	for time.Now().Before(deadline) {
		if err := sleepUntil(ctx, g1HandPoll); err != nil {
			h.stopLocked()
			return last, err
		}
		st, err := h.hand.Status()
		if err != nil {
			return last, err
		}
		if st.Positions == last.Positions {
			stillCount++
		} else {
			stillCount = 0
		}
		last = st
		// Require a few still reads in a row so we don't stop before the
		// fingers have started moving.
		if stillCount >= 3 {
			break
		}
	}
	return last, nil
}

// stopLocked holds the fingers where they are by commanding their current
// positions.
func (h *g1Hand) stopLocked() error {
	st, err := h.hand.Status()
	if err != nil {
		return err
	}
	var targets [brainCoNumFingers]int
	for i, p := range st.Positions {
		targets[i] = (p + 5) / 10
	}
	return h.hand.SetTargetsPercent(targets)
}

func (h *g1Hand) Open(ctx context.Context, extra map[string]interface{}) error {
	_, err := h.moveTo(ctx, g1HandPoses["open"])
	return err
}

// Grab closes to the grab pose and reports whether any finger was stopped
// short by an object.
func (h *g1Hand) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	st, err := h.moveTo(ctx, h.grabPose)
	if err != nil {
		return false, err
	}
	return h.blocked(st, h.grabPose), nil
}

func (h *g1Hand) blocked(st BrainCoStatus, targets [brainCoNumFingers]int) bool {
	// Skip the thumb rotation axis: it moves sideways, not into objects.
	for i := 0; i < brainCoNumFingers; i++ {
		if i == 1 {
			continue
		}
		if targets[i]*10-st.Positions[i] > g1HandBlockedMargin {
			return true
		}
	}
	return false
}

func (h *g1Hand) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	st, err := h.hand.Status()
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	meta := map[string]interface{}{}
	for i, name := range brainCoFingerNames {
		meta[name+"_position"] = st.Positions[i]
		meta[name+"_current"] = st.Currents[i]
	}
	return gripper.HoldingStatus{IsHoldingSomething: h.blocked(st, h.grabPose), Meta: meta}, nil
}

func (h *g1Hand) Stop(ctx context.Context, extra map[string]interface{}) error {
	return h.stopLocked()
}

func (h *g1Hand) IsMoving(ctx context.Context) (bool, error) {
	st, err := h.hand.Status()
	if err != nil {
		return false, err
	}
	for _, s := range st.Speeds {
		if s != 0 {
			return true, nil
		}
	}
	return false, nil
}

// DoCommand:
//
//	{"pose": "point"}                         move to a named pose (see "poses")
//	{"fingers": {"index": 0, "thumb": 50}}    percent closed per finger; others hold
//	{"get_fingers": true}                     positions (percent closed), currents, status
//	{"poses": true}                           list named poses
func (h *g1Hand) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if name, ok := cmd["pose"].(string); ok {
		targets, ok := g1HandPoses[name]
		if !ok {
			return nil, fmt.Errorf("unknown pose %q; try {\"poses\": true}", name)
		}
		st, err := h.moveTo(ctx, targets)
		if err != nil {
			return nil, err
		}
		return fingersResult(st), nil
	}

	if raw, ok := cmd["fingers"].(map[string]interface{}); ok {
		st, err := h.hand.Status()
		if err != nil {
			return nil, err
		}
		var targets [brainCoNumFingers]int
		for i, p := range st.Positions {
			targets[i] = (p + 5) / 10
		}
		for name, v := range raw {
			idx := -1
			for i, n := range brainCoFingerNames {
				if n == name {
					idx = i
				}
			}
			if idx < 0 {
				return nil, fmt.Errorf("unknown finger %q; fingers are %v", name, brainCoFingerNames)
			}
			pct, err := numericToInt(v)
			if err != nil {
				return nil, fmt.Errorf("finger %q: %w", name, err)
			}
			if pct < 0 || pct > 100 {
				return nil, fmt.Errorf("finger %q: %d is outside 0..100", name, pct)
			}
			targets[idx] = pct
		}
		st, err = h.moveTo(ctx, targets)
		if err != nil {
			return nil, err
		}
		return fingersResult(st), nil
	}

	if _, ok := cmd["get_fingers"]; ok {
		st, err := h.hand.Status()
		if err != nil {
			return nil, err
		}
		return fingersResult(st), nil
	}

	if _, ok := cmd["poses"]; ok {
		names := make([]interface{}, 0, len(g1HandPoses))
		keys := make([]string, 0, len(g1HandPoses))
		for k := range g1HandPoses {
			keys = append(keys, k)
		}
		sort.Strings(keys)
		for _, k := range keys {
			names = append(names, k)
		}
		return map[string]interface{}{"poses": names}, nil
	}

	return nil, errors.New(`unknown command; supported: pose, fingers, get_fingers, poses`)
}

// fingersResult reports positions as percent closed, plus currents and
// motor status, keyed by finger name.
func fingersResult(st BrainCoStatus) map[string]interface{} {
	positions := map[string]interface{}{}
	currents := map[string]interface{}{}
	status := map[string]interface{}{}
	for i, name := range brainCoFingerNames {
		positions[name] = float64(st.Positions[i]) / 10
		currents[name] = st.Currents[i]
		status[name] = st.Motor[i]
	}
	return map[string]interface{}{"percent_closed": positions, "currents": currents, "motor_status": status}
}

func (h *g1Hand) Kinematics(ctx context.Context) (referenceframe.Model, error) { return h.model, nil }

func (h *g1Hand) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (h *g1Hand) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return nil
}

func (h *g1Hand) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func (h *g1Hand) Close(ctx context.Context) error {
	return h.hand.Close()
}

// sleepUntil waits for d or until ctx is done.
func sleepUntil(ctx context.Context, d time.Duration) error {
	timer := time.NewTimer(d)
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return nil
	}
}
