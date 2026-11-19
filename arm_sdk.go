package main

/*
#include "dds_unitree.h"
#include <stdlib.h>
#include <string.h>
*/
import "C"

import (
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"
	"unsafe"

	"go.viam.com/rdk/logging"
)

// G1 joint indices in the unitree_hg motor array (35 motors total).
// See unitree_sdk2 example/g1/high_level/g1_arm_sdk_dds_example.cpp.
const (
	G1JointLeftHipPitch        = 0
	G1JointLeftHipRoll         = 1
	G1JointLeftHipYaw          = 2
	G1JointLeftKnee            = 3
	G1JointLeftAnklePitch      = 4
	G1JointLeftAnkleRoll       = 5
	G1JointRightHipPitch       = 6
	G1JointRightHipRoll        = 7
	G1JointRightHipYaw         = 8
	G1JointRightKnee           = 9
	G1JointRightAnklePitch     = 10
	G1JointRightAnkleRoll      = 11
	G1JointWaistYaw            = 12
	G1JointWaistRoll           = 13 // locked on 23/29-DoF G1
	G1JointWaistPitch          = 14 // locked on 23/29-DoF G1
	G1JointLeftShoulderPitch   = 15
	G1JointLeftShoulderRoll    = 16
	G1JointLeftShoulderYaw     = 17
	G1JointLeftElbow           = 18
	G1JointLeftWristRoll       = 19
	G1JointLeftWristPitch      = 20
	G1JointLeftWristYaw        = 21
	G1JointRightShoulderPitch  = 22
	G1JointRightShoulderRoll   = 23
	G1JointRightShoulderYaw    = 24
	G1JointRightElbow          = 25
	G1JointRightWristRoll      = 26
	G1JointRightWristPitch     = 27
	G1JointRightWristYaw       = 28

	// Motor 29 is not a real joint - its q field carries the arm_sdk
	// "weight": 0 = sport-mode controls the arms, 1 = arm_sdk fully overrides.
	G1ArmSDKWeightIndex = 29

	G1NumMotors = 35

	// Number of DoF on a single G1 arm (shoulder pitch/roll/yaw + elbow +
	// wrist roll/pitch/yaw).
	G1ArmDoF = 7
)

// armSDK is a process-wide singleton that owns the rt/arm_sdk publisher and
// the rt/lowstate subscriber. The two arm components (left + right) share
// the same writer/reader and use disjoint slices of the motor array.
//
// Lifecycle is reference counted: the first arm to start it allocates the
// resources, the last arm to close releases them.
type armSDK struct {
	mu sync.Mutex

	refs int

	writer C.dds_entity_t
	reader C.dds_entity_t

	// commanded holds the target for every motor. arm_sdk drives the upper
	// body: each arm component writes its own arm slice, and the waist holds
	// the pose it had when arm_sdk engaged. Legs and motors 30-34 stay zero
	// (uncommanded), which leaves them to the balance controller.
	commanded [G1NumMotors]C.unitree_hg_motor_cmd_t

	// weight is the arm_sdk blending weight (motor 29) being published now;
	// target is where the current or last ramp is heading. target == 0 means
	// released: once weight reaches 0, the publisher sends a final zero-weight
	// frame and closes the writer, handing the upper body back to sport.
	weight float32
	target float32

	// rampMu serializes engage/release ramps, so an arm move during a
	// release waits for it and then engages again instead of being lost.
	rampMu sync.Mutex

	// gravityScale is how much of the modeled gravity torque the upper-body
	// commands carry as feedforward: 0 off, 1 the full model (gravity.go).
	gravityScale float64

	// logger reports engage/release, motor error codes and lowstate gaps, so
	// a fault can be traced to what arm_sdk was doing. Nil in tests.
	logger logging.Logger
	// lastMotorCode and lastCodeLog track each motor's error code from
	// lowstate, to log changes (at most once a second per motor).
	lastMotorCode [G1NumMotors]uint32
	lastCodeLog   [G1NumMotors]time.Time
	// stateStale is set while lowstate has stopped arriving.
	stateStale bool
	// safetyRelease is set while arm_sdk fades out because the robot tipped.
	safetyRelease bool
	// lastYieldLog and lastTorqueLog rate-limit those warnings per motor.
	lastYieldLog  [G1NumMotors]time.Time
	lastTorqueLog [G1NumMotors]time.Time

	// latestState is updated by the lowstate poller. Reads must hold the
	// mutex; the value is plain-old-data so we copy it out.
	latestState   C.unitree_hg_lowstate_t
	hasState      atomic.Bool
	lastStateTime time.Time

	stopCh chan struct{}
	wg     sync.WaitGroup
}

var (
	armSDKMu       sync.Mutex
	armSDKInstance *armSDK
)

// Gains for the upper-body joints arm_sdk holds, from Unitree's
// g1_arm7_sdk_dds_example. Arm components override the arms' gains with their
// own kp/kd on their first move.
const (
	upperBodyKp = 60
	upperBodyKd = 1.5
)

// Safety limits while arm_sdk drives the upper body.
const (
	// maxEngagedTilt is how far the pelvis may tip from upright (15°) before
	// arm_sdk releases, and refuses to engage: past it the robot is falling or
	// hanging in a harness, and holding the waist rigid fights whatever holds it.
	maxEngagedTilt = 15 * math.Pi / 180
	// safetyReleaseStep is the weight drop per 20 ms tick in a safety release
	// (0.2 s from full weight).
	safetyReleaseStep = 0.1
	// waistYield is how far (rad) a waist joint may be pushed off its hold
	// before the target follows it, capping the hold at kp × 0.15 = 9 N·m.
	waistYield = 0.15
	// torqueWarnShare logs commands above this share of a motor's rated torque.
	torqueWarnShare = 0.8
)

// releaseArmSDK hands the arms and waist back to the sport controller if
// arm_sdk is engaged, and returns once it has. Sport-service commands that
// change mode or move the arms (ready_to_move, FSM changes, gestures) need
// this: while rt/arm_sdk is open the firmware rejects FSM transitions, and a
// gesture would fight arm_sdk's targets. Safe to call when no arm component
// exists or arm_sdk is already released.
func releaseArmSDK() error {
	armSDKMu.Lock()
	a := armSDKInstance
	armSDKMu.Unlock()
	if a == nil {
		return nil
	}
	return a.disengage()
}

// armSDKEngaged reports whether arm_sdk is engaged or engaging.
func armSDKEngaged() bool {
	armSDKMu.Lock()
	a := armSDKInstance
	armSDKMu.Unlock()
	if a == nil {
		return false
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.target > 0 || a.weight > 0
}

// getArmSDK returns the shared arm SDK singleton, creating it on first use.
//
// The rt/arm_sdk DDS writer is NOT created here — it is lazily created
// the first time setWeight is called with weight > 0. Just registering
// the writer is enough for the firmware to detect an arm_sdk client and
// enter low-level mode, which blocks all sport-service FSM transitions.
// By deferring writer creation, the sport service stays functional
// until the arms are actively needed.
func getArmSDK(logger logging.Logger) (*armSDK, error) {
	armSDKMu.Lock()
	defer armSDKMu.Unlock()

	if armSDKInstance != nil {
		armSDKInstance.mu.Lock()
		armSDKInstance.refs++
		armSDKInstance.mu.Unlock()
		return armSDKInstance, nil
	}

	a := &armSDK{
		refs:         1,
		weight:       0,
		gravityScale: 1,
		logger:       logger,
		stopCh:       make(chan struct{}),
	}

	a.initCommands()

	// Writer is created lazily in ensureWriter() when weight > 0.

	stateTopic := C.CString("rt/lowstate")
	defer C.free(unsafe.Pointer(stateTopic))
	if rc := C.unitree_dds_subscribe(stateTopic, 1 /* lowstate */, &a.reader); rc != 0 {
		return nil, fmt.Errorf("subscribe rt/lowstate failed (rc=%d)", rc)
	}

	a.wg.Add(1)
	go a.run()

	armSDKInstance = a
	return a, nil
}

// initCommands sets up the commands arm_sdk publishes, following Unitree's
// g1_arm7_sdk_dds_example: the 17 upper-body motors (waist 12-14 and arms
// 15-28) are enabled with holding gains, and their targets are seeded from
// the measured pose each time arm_sdk engages. Every other motor (legs 0-11,
// 30-34) stays zero, so the balance controller keeps the legs.
func (a *armSDK) initCommands() {
	a.commanded = [G1NumMotors]C.unitree_hg_motor_cmd_t{}
	for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
		a.commanded[i].mode = 1
		a.commanded[i].kp = upperBodyKp
		a.commanded[i].kd = upperBodyKd
	}
}

// seedUpperBodyLocked sets every upper-body target to the measured position,
// so engaging arm_sdk holds the current pose instead of pulling toward old
// targets or zero. Caller must hold a.mu.
func (a *armSDK) seedUpperBodyLocked() {
	for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
		a.commanded[i].q = a.latestState.motor_state[i].q
		a.commanded[i].dq = 0
	}
}

// release decrements the reference count and tears down the shared resources
// when no arm components are left.
func (a *armSDK) release() {
	armSDKMu.Lock()
	defer armSDKMu.Unlock()

	a.mu.Lock()
	a.refs--
	if a.refs > 0 {
		a.mu.Unlock()
		return
	}
	a.mu.Unlock()

	close(a.stopCh)
	a.wg.Wait()

	if a.writer != 0 {
		C.unitree_dds_close_writer(a.writer)
		a.writer = 0
	}
	if a.reader != 0 {
		C.unitree_dds_close_subscriber(a.reader)
		a.reader = 0
	}

	armSDKInstance = nil
}

// setArmCommand updates the commanded position/gains for the given motor
// indices. Pass equal-length slices for indices, q, kp, kd; dq and tau are
// set to zero (typical for position control).
func (a *armSDK) setArmCommand(indices []int, q, kp, kd []float32) error {
	if len(q) != len(indices) || len(kp) != len(indices) || len(kd) != len(indices) {
		return fmt.Errorf("setArmCommand: slice length mismatch")
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	for i, idx := range indices {
		if idx < 0 || idx >= G1NumMotors {
			return fmt.Errorf("setArmCommand: index %d out of range", idx)
		}
		a.commanded[idx].mode = 1
		a.commanded[idx].q = C.float(q[i])
		a.commanded[idx].dq = 0
		a.commanded[idx].tau = 0
		a.commanded[idx].kp = C.float(kp[i])
		a.commanded[idx].kd = C.float(kd[i])
	}
	return nil
}

// ensureWriter lazily creates the rt/arm_sdk DDS writer. Must be called
// with a.mu held. Returns an error if the writer can't be created.
func (a *armSDK) ensureWriter() error {
	if a.writer != 0 {
		return nil
	}
	cmdTopic := C.CString("rt/arm_sdk")
	defer C.free(unsafe.Pointer(cmdTopic))
	if rc := C.unitree_dds_create_lowcmd_writer(cmdTopic, &a.writer); rc != 0 {
		return fmt.Errorf("create arm_sdk writer failed (rc=%d)", rc)
	}
	return nil
}

// rampWeight gradually transitions the arm_sdk weight from its current
// value to target over duration. This avoids jarring torque spikes when
// switching between sport and arm_sdk control. Steps at ~50 Hz (20ms).
// Ramping up from released seeds the upper body from the measured pose.
// Ramping to 0 releases arm_sdk: the publisher then sends a final
// zero-weight frame and closes the writer.
func (a *armSDK) rampWeight(target float32, duration time.Duration) error {
	a.rampMu.Lock()
	defer a.rampMu.Unlock()

	a.mu.Lock()
	if target > 0 {
		if err := a.engageLocked(target); err != nil {
			a.mu.Unlock()
			return err
		}
	} else {
		a.target = 0
	}
	start := a.weight
	scale := a.gravityScale
	a.mu.Unlock()
	switch {
	case target > 0 && start == 0:
		a.logInfof("arm_sdk engaging: holding the current upper-body pose (gravity compensation scale %.2f)", scale)
	case target == 0 && start > 0:
		a.logInfof("arm_sdk releasing from weight %.2f", start)
	}

	steps := int(duration / (20 * time.Millisecond))
	if steps < 1 {
		steps = 1
	}
	for i := 1; i <= steps; i++ {
		t := float32(i) / float32(steps)
		w := start + (target-start)*t
		a.mu.Lock()
		if a.target != target {
			// A safety release took over; let it finish.
			a.mu.Unlock()
			return fmt.Errorf("arm_sdk released for safety: the pelvis tilted past %.0f°", maxEngagedTilt*180/math.Pi)
		}
		a.weight = w
		a.mu.Unlock()
		time.Sleep(20 * time.Millisecond)
	}
	return nil
}

// engageLocked prepares arm_sdk to publish toward target > 0: it needs
// lowstate, seeds the upper body when starting from released (weight 0), and
// opens the writer. Caller must hold a.mu.
func (a *armSDK) engageLocked(target float32) error {
	if !a.hasState.Load() {
		return fmt.Errorf("cannot engage arm_sdk: no lowstate received yet")
	}
	if tilt := a.tiltLocked(); tilt > maxEngagedTilt {
		return fmt.Errorf("not engaging arm_sdk: the pelvis is tilted %.0f° (limit %.0f°); stand the robot upright first",
			tilt*180/math.Pi, maxEngagedTilt*180/math.Pi)
	}
	if a.weight == 0 {
		a.seedUpperBodyLocked()
	}
	if err := a.ensureWriter(); err != nil {
		return err
	}
	a.target = target
	return nil
}

// setWeight sets the arm_sdk blending weight (0..1) at once. When non-zero,
// arm_sdk motor commands override the sport controller for the upper body;
// 0 releases arm_sdk.
func (a *armSDK) setWeight(w float32) error {
	a.rampMu.Lock()
	defer a.rampMu.Unlock()
	a.mu.Lock()
	defer a.mu.Unlock()
	if w < 0 {
		w = 0
	} else if w > 1 {
		w = 1
	}
	if w > 0 {
		if err := a.engageLocked(w); err != nil {
			return err
		}
	} else {
		a.target = 0
	}
	a.weight = w
	return nil
}

// ensureEngaged ramps the arm_sdk weight up to 1 if arm_sdk is released or
// releasing, holding the current pose (see engageLocked). It is a no-op once
// engaged or engaging, so it is safe to call on every move: this is what lets
// UI joint-jogging work without a manual "engage" first. During a release it
// waits for the release to finish, then engages again.
func (a *armSDK) ensureEngaged() error {
	a.mu.Lock()
	engaged := a.target > 0
	a.mu.Unlock()
	if engaged {
		return nil
	}
	return a.rampWeight(1, 1*time.Second)
}

// disengage fades arm_sdk out and waits until the publisher has sent the
// final zero-weight frame and closed the writer, so sport-service commands
// that follow aren't rejected.
func (a *armSDK) disengage() error {
	a.mu.Lock()
	idle := a.target == 0 && a.weight == 0 && a.writer == 0
	a.mu.Unlock()
	if idle {
		return nil
	}
	if err := a.rampWeight(0, 1*time.Second); err != nil {
		return err
	}
	for deadline := time.Now().Add(1 * time.Second); time.Now().Before(deadline); {
		a.mu.Lock()
		closed := a.writer == 0
		a.mu.Unlock()
		if closed {
			return nil
		}
		time.Sleep(20 * time.Millisecond)
	}
	return fmt.Errorf("arm_sdk did not release within 1s")
}

// jointPosition returns the latest measured position of the given motor
// index, or (0, false) if no lowstate has been received yet.
func (a *armSDK) jointPosition(idx int) (float32, bool) {
	if !a.hasState.Load() {
		return 0, false
	}
	if idx < 0 || idx >= G1NumMotors {
		return 0, false
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	return float32(a.latestState.motor_state[idx].q), true
}

// jointPositions returns the latest measured positions for the given indices.
// If no state has been received yet, returns (nil, false).
func (a *armSDK) jointPositions(indices []int) ([]float32, bool) {
	if !a.hasState.Load() {
		return nil, false
	}
	out := make([]float32, len(indices))
	a.mu.Lock()
	defer a.mu.Unlock()
	for i, idx := range indices {
		if idx < 0 || idx >= G1NumMotors {
			return nil, false
		}
		out[i] = float32(a.latestState.motor_state[idx].q)
	}
	return out, true
}

// run is the publish/poll loop. Reads LowState every 20 ms. Publishes
// LowCmd only while arm_sdk is engaged (weight > 0), plus one final
// zero-weight frame on release, after which the writer is closed: an open
// rt/arm_sdk writer, even at weight=0, makes the sport service refuse FSM
// transitions (status 7301). Staying silent when released lets sport retain
// full control of the robot.
func (a *armSDK) run() {
	defer a.wg.Done()
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-a.stopCh:
			// Best-effort handoff to sport: if we were driving the arms,
			// publish a final weight=0 frame so the firmware ramps back
			// to sport-owned arms before we go silent.
			a.mu.Lock()
			writer := a.writer
			wasActive := a.weight > 0 && writer != 0
			a.weight = 0
			a.target = 0
			a.commanded[G1ArmSDKWeightIndex].q = 0
			cmd := a.snapshotLocked()
			a.mu.Unlock()
			if wasActive {
				C.unitree_dds_publish_lowcmd(writer, &cmd)
			}
			return
		case <-ticker.C:
		}

		a.mu.Lock()
		cmd, writer, closeAfter, events := a.nextFrameLocked()
		a.mu.Unlock()
		for _, e := range events {
			a.logWarnf("%s", e)
		}

		if writer != 0 {
			C.unitree_dds_publish_lowcmd(writer, &cmd)
		}
		if closeAfter {
			C.unitree_dds_close_writer(writer)
			a.logInfof("arm_sdk released: sent a zero-weight frame and closed the writer")
		}

		now := time.Now()
		var ls C.unitree_hg_lowstate_t
		if rc := C.unitree_dds_take_lowstate(a.reader, 0, &ls); rc == 0 {
			a.mu.Lock()
			if a.stateStale {
				a.stateStale = false
				a.logInfof("rt/lowstate resumed after %v", now.Sub(a.lastStateTime).Round(time.Millisecond))
			}
			a.latestState = ls
			a.lastStateTime = now
			events := a.motorCodeEventsLocked(now)
			a.mu.Unlock()
			a.hasState.Store(true)
			for _, e := range events {
				a.logWarnf("%s", e)
			}
		} else if a.hasState.Load() {
			a.mu.Lock()
			if !a.stateStale && now.Sub(a.lastStateTime) > time.Second {
				a.stateStale = true
				a.logWarnf("no rt/lowstate for %v (arm_sdk weight %.2f): the motion controller may have stopped or restarted",
					now.Sub(a.lastStateTime).Round(time.Millisecond), a.weight)
			}
			a.mu.Unlock()
		}
	}
}

// motorCodeEventsLocked compares each motor's error code in the latest lowstate
// with the last one seen and describes the changes, at most once a second per
// motor. The first lowstate reports every nonzero code. Caller must hold a.mu.
func (a *armSDK) motorCodeEventsLocked(now time.Time) []string {
	var events []string
	for i := range g1MotorJoints {
		m := a.latestState.motor_state[i]
		code := uint32(m.motorstate)
		if code == a.lastMotorCode[i] {
			continue
		}
		if now.Sub(a.lastCodeLog[i]) >= time.Second {
			events = append(events, fmt.Sprintf(
				"motor %d (%s) error code 0x%x -> 0x%x: temperature %d/%d C, estimated torque %.1f N·m, q %.3f rad; arm_sdk weight %.2f, gravity scale %.2f",
				i, g1MotorJoints[i], a.lastMotorCode[i], code, int(m.temperature[0]), int(m.temperature[1]),
				float64(m.tau_est), float64(m.q), a.weight, a.gravityScale))
			a.lastCodeLog[i] = now
		}
		a.lastMotorCode[i] = code
	}
	return events
}

// motorStatusReport returns the latest rt/lowstate per motor (angle, velocity,
// estimated torque, temperatures, error code), the IMU attitude, and arm_sdk's
// state. It needs an arm or body component, which own the lowstate reader.
func motorStatusReport() (map[string]interface{}, error) {
	armSDKMu.Lock()
	a := armSDKInstance
	armSDKMu.Unlock()
	if a == nil {
		return nil, fmt.Errorf("no g1 arm or body component is running; they read rt/lowstate")
	}
	if !a.hasState.Load() {
		return nil, fmt.Errorf("no rt/lowstate received yet")
	}
	round := func(v float64, places float64) float64 {
		f := math.Pow(10, places)
		return math.Round(v*f) / f
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	motors := map[string]interface{}{}
	var faults []interface{}
	for i, name := range g1MotorJoints {
		m := a.latestState.motor_state[i]
		code := uint32(m.motorstate)
		motors[name] = map[string]interface{}{
			"index":         float64(i),
			"mode":          float64(m.mode),
			"q_rad":         round(float64(m.q), 4),
			"dq_rad_s":      round(float64(m.dq), 3),
			"tau_est_nm":    round(float64(m.tau_est), 2),
			"temperature_c": []interface{}{float64(m.temperature[0]), float64(m.temperature[1])},
			"voltage":       round(float64(m.vol), 1),
			"error_code":    fmt.Sprintf("0x%x", code),
		}
		if code != 0 {
			faults = append(faults, name)
		}
	}
	rpy := a.latestState.imu_state.rpy
	return map[string]interface{}{
		"motors":           motors,
		"motors_with_code": faults,
		"imu_rpy_rad":      []interface{}{round(float64(rpy[0]), 4), round(float64(rpy[1]), 4), round(float64(rpy[2]), 4)},
		"lowstate_age_ms":  float64(time.Since(a.lastStateTime).Milliseconds()),
		"arm_sdk": map[string]interface{}{
			"weight":        round(float64(a.weight), 2),
			"target":        round(float64(a.target), 2),
			"writer_open":   a.writer != 0,
			"gravity_scale": a.gravityScale,
		},
	}, nil
}

func (a *armSDK) logInfof(format string, args ...interface{}) {
	if a.logger != nil {
		a.logger.Infof(format, args...)
	}
}

func (a *armSDK) logWarnf(format string, args ...interface{}) {
	if a.logger != nil {
		a.logger.Warnf(format, args...)
	}
}

// nextFrameLocked builds this tick's LowCmd and returns the writer to publish
// it on (0: publish nothing) and whether to close that writer afterwards.
// While engaged it publishes every tick. Once a release has brought the
// weight to 0, it publishes one final zero-weight frame and closes the writer
// (clearing a.writer, so the next engage opens a new one). Caller must hold
// a.mu.
//
// While engaged it also applies the safety limits, returning a description of
// each for the log: it releases if the pelvis tips past maxEngagedTilt, lets
// the waist yield when forced off its hold, and flags commands near a motor's
// rated torque.
func (a *armSDK) nextFrameLocked() (cmd C.unitree_hg_lowcmd_t, writer C.dds_entity_t, closeAfter bool, events []string) {
	now := time.Now()
	if a.hasState.Load() && (a.target > 0 || a.weight > 0) {
		if tilt := a.tiltLocked(); tilt > maxEngagedTilt && a.target > 0 {
			a.target = 0
			a.safetyRelease = true
			events = append(events, fmt.Sprintf("pelvis tilted %.0f° (limit %.0f°): releasing arm_sdk",
				tilt*180/math.Pi, maxEngagedTilt*180/math.Pi))
		}
		if a.safetyRelease {
			a.weight = float32(math.Max(0, float64(a.weight)-safetyReleaseStep))
		}
		events = append(events, a.yieldWaistLocked(now)...)
	}

	a.commanded[G1ArmSDKWeightIndex].q = C.float(a.weight)
	cmd = a.snapshotLocked()
	if a.weight > 0 {
		events = append(events, a.torqueWarningsLocked(&cmd, now)...)
	}
	switch {
	case a.writer == 0:
		return cmd, 0, false, events
	case a.weight > 0:
		return cmd, a.writer, false, events
	case a.target == 0:
		writer = a.writer
		a.writer = 0
		a.safetyRelease = false
		return cmd, writer, true, events
	default:
		// Engaging, before the first ramp step: nothing to publish yet.
		return cmd, 0, false, events
	}
}

// tiltLocked returns the pelvis tilt from upright, in radians, from the IMU.
// Caller must hold a.mu.
func (a *armSDK) tiltLocked() float64 {
	q := a.latestState.imu_state.quaternion
	return pelvisTilt(float64(q[0]), float64(q[1]), float64(q[2]), float64(q[3]))
}

// yieldWaistLocked moves each waist target to within waistYield of where the
// joint is, so a waist pushed off its hold (by the harness, the legs, or a
// person) isn't fought with ever more torque. Caller must hold a.mu.
func (a *armSDK) yieldWaistLocked(now time.Time) []string {
	var events []string
	for i := G1JointWaistYaw; i <= G1JointWaistPitch; i++ {
		measured := float64(a.latestState.motor_state[i].q)
		target := float64(a.commanded[i].q)
		if math.Abs(target-measured) <= waistYield {
			continue
		}
		a.commanded[i].q = C.float(measured + math.Copysign(waistYield, target-measured))
		if now.Sub(a.lastYieldLog[i]) >= 5*time.Second {
			a.lastYieldLog[i] = now
			events = append(events, fmt.Sprintf("%s pushed %.2f rad off its hold; the hold now follows it",
				g1MotorJoints[i], measured-target))
		}
	}
	return events
}

// torqueWarningsLocked flags upper-body motors whose commanded torque (the
// position hold plus gravity feedforward) is above torqueWarnShare of the
// motor's rated torque, at most every 5 s per motor. Caller must hold a.mu.
func (a *armSDK) torqueWarningsLocked(cmd *C.unitree_hg_lowcmd_t, now time.Time) []string {
	model, err := upperBodyGravityModel()
	if err != nil {
		return nil
	}
	var events []string
	for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
		m, s := cmd.motor_cmd[i], a.latestState.motor_state[i]
		torque := float64(m.kp)*float64(m.q-s.q) - float64(m.kd)*float64(s.dq) + float64(m.tau)
		rated := model.effort[i]
		if rated <= 0 || math.Abs(torque) <= torqueWarnShare*rated || now.Sub(a.lastTorqueLog[i]) < 5*time.Second {
			continue
		}
		a.lastTorqueLog[i] = now
		events = append(events, fmt.Sprintf("%s commanded %.1f N·m, %.0f%% of its %.0f N·m rating (hold %.1f, gravity %.1f)",
			g1MotorJoints[i], torque, 100*math.Abs(torque)/rated, rated, torque-float64(m.tau), float64(m.tau)))
	}
	return events
}

// gravityTorquesLocked returns the feedforward torques that hold the measured
// upper-body pose against gravity, scaled by gravityScale and capped per joint
// (see gravity.go). ok is false when there is nothing to send: no lowstate yet,
// compensation off, or no model. Caller must hold a.mu.
func (a *armSDK) gravityTorquesLocked() (tau [G1NumMotors]float64, ok bool) {
	if !a.hasState.Load() || a.gravityScale <= 0 {
		return tau, false
	}
	model, err := upperBodyGravityModel()
	if err != nil {
		return tau, false
	}
	var q [G1NumMotors]float64
	for i := range q {
		q[i] = float64(a.latestState.motor_state[i].q)
	}
	imu := a.latestState.imu_state.quaternion
	gravity := pelvisGravity(float64(imu[0]), float64(imu[1]), float64(imu[2]), float64(imu[3]))
	tau = model.holdingTorques(&q, gravity)
	for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
		limit := model.torqueLimit(i)
		tau[i] = math.Max(-limit, math.Min(limit, a.gravityScale*tau[i]))
	}
	return tau, true
}

// setGravityScale sets how much of the modeled gravity torque is sent: 0 turns
// compensation off, 1 is the full model.
func (a *armSDK) setGravityScale(scale float64) error {
	if scale < 0 || scale > 1.5 || math.IsNaN(scale) {
		return fmt.Errorf("gravity compensation scale must be between 0 and 1.5, got %v", scale)
	}
	a.mu.Lock()
	old := a.gravityScale
	a.gravityScale = scale
	a.mu.Unlock()
	a.logInfof("gravity compensation scale %.2f -> %.2f", old, scale)
	return nil
}

// gravityStatus returns the gravity compensation scale and, for each upper-body
// joint, the feedforward torque it adds at the current pose (N·m).
func (a *armSDK) gravityStatus() (float64, map[string]float64, error) {
	if !a.hasState.Load() {
		return 0, nil, fmt.Errorf("no lowstate received yet")
	}
	if _, err := upperBodyGravityModel(); err != nil {
		return 0, nil, fmt.Errorf("gravity model: %w", err)
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	tau, _ := a.gravityTorquesLocked()
	torques := map[string]float64{}
	for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
		torques[g1MotorJoints[i]] = math.Round(tau[i]*100) / 100
	}
	return a.gravityScale, torques, nil
}

// snapshotLocked builds a LowCmd from the current commanded array. Caller
// must hold a.mu. mode_pr and mode_machine are echoed from the latest
// LowState (required by firmware — sending 0 causes body to drop into
// safety mode). Upper-body targets are fixed between moves rather than
// echoed from lowstate: a target that follows the measured position has no
// stiffness, so the joint sags. CRC32 is computed over the entire struct.
func (a *armSDK) snapshotLocked() C.unitree_hg_lowcmd_t {
	var cmd C.unitree_hg_lowcmd_t

	// Echo mode fields from the latest lowstate.
	if a.hasState.Load() {
		cmd.mode_pr = a.latestState.mode_pr
		cmd.mode_machine = a.latestState.mode_machine
	}

	for i := 0; i < G1NumMotors; i++ {
		cmd.motor_cmd[i] = a.commanded[i]
	}

	// Hold the upper body against gravity: feedforward torque, so the joints
	// don't have to sag off their targets before kp pushes back.
	if tau, ok := a.gravityTorquesLocked(); ok {
		for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
			cmd.motor_cmd[i].tau = C.float(tau[i])
		}
	}

	// CRC32 over the entire struct excluding the crc field itself.
	cmd.crc = crc32Unitree(unsafe.Slice((*byte)(unsafe.Pointer(&cmd)), unsafe.Sizeof(cmd)-4))
	return cmd
}

// crc32Unitree computes the CRC-32 used by Unitree's LowCmd/LowState
// (polynomial 0x04c11db7, no bit-reversal — matches the SDK2 example's
// Crc32Core operating on uint32 words).
func crc32Unitree(data []byte) C.uint32_t {
	const poly = 0x04c11db7
	var crc uint32 = 0

	// Process 4 bytes at a time (uint32 words), big-endian interpretation
	// matching the SDK2's (uint32_t*) cast.
	for i := 0; i+3 < len(data); i += 4 {
		word := uint32(data[i]) | uint32(data[i+1])<<8 |
			uint32(data[i+2])<<16 | uint32(data[i+3])<<24
		crc ^= word
		for bit := 0; bit < 32; bit++ {
			if crc&0x80000000 != 0 {
				crc = (crc << 1) ^ poly
			} else {
				crc <<= 1
			}
		}
	}
	return C.uint32_t(crc)
}
