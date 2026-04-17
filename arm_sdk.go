package main

/*
#include "dds_unitree.h"
#include <stdlib.h>
#include <string.h>
*/
import "C"

import (
	"fmt"
	"sync"
	"sync/atomic"
	"time"
	"unsafe"
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

// LeftArmJointIndices and RightArmJointIndices give the motor-array indices
// for each arm in shoulder-to-wrist order.
var (
	LeftArmJointIndices = []int{
		G1JointLeftShoulderPitch,
		G1JointLeftShoulderRoll,
		G1JointLeftShoulderYaw,
		G1JointLeftElbow,
		G1JointLeftWristRoll,
		G1JointLeftWristPitch,
		G1JointLeftWristYaw,
	}
	RightArmJointIndices = []int{
		G1JointRightShoulderPitch,
		G1JointRightShoulderRoll,
		G1JointRightShoulderYaw,
		G1JointRightElbow,
		G1JointRightWristRoll,
		G1JointRightWristPitch,
		G1JointRightWristYaw,
	}
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

	// commanded holds the most recent target for every arm motor. Each arm
	// component writes its own slice; the publisher loop sends the union.
	commanded [G1NumMotors]C.unitree_hg_motor_cmd_t

	// weight is the arm_sdk blending weight (motor 29). We hold it at 1.0
	// while at least one arm is active so arm_sdk fully owns the arms.
	weight float32

	// latestState is updated by the lowstate poller. Reads must hold the
	// mutex; the value is plain-old-data so we copy it out.
	latestState   C.unitree_hg_lowstate_t
	hasState      atomic.Bool
	lastStateTime time.Time

	// paused gates the LowCmd publish in run(). Set while the G1 generic
	// component is issuing sport-service FSM transitions: the firmware
	// silently rejects sport stand-up transitions while rt/arm_sdk is
	// actively publishing, even with weight=0.
	paused atomic.Bool

	stopCh chan struct{}
	wg     sync.WaitGroup
}

var (
	armSDKMu       sync.Mutex
	armSDKInstance *armSDK
)

// pauseActiveArmSDK looks up the process-wide armSDK singleton (if any)
// and gates its LowCmd publisher. Returns a resume func; safe to call
// whether or not an armSDK exists. Intended use: the G1 generic component
// pauses arm_sdk publishing around sport-service FSM transitions, which
// the firmware silently rejects while rt/arm_sdk traffic is active.
func pauseActiveArmSDK() func() {
	armSDKMu.Lock()
	a := armSDKInstance
	armSDKMu.Unlock()
	if a == nil {
		return func() {}
	}
	a.paused.Store(true)
	return func() { a.paused.Store(false) }
}

// getArmSDK returns the shared arm SDK singleton, creating it on first use.
//
// The rt/arm_sdk DDS writer is NOT created here — it is lazily created
// the first time setWeight is called with weight > 0. Just registering
// the writer is enough for the firmware to detect an arm_sdk client and
// enter low-level mode, which blocks all sport-service FSM transitions.
// By deferring writer creation, the sport service stays functional
// until the arms are actively needed.
func getArmSDK() (*armSDK, error) {
	armSDKMu.Lock()
	defer armSDKMu.Unlock()

	if armSDKInstance != nil {
		armSDKInstance.mu.Lock()
		armSDKInstance.refs++
		armSDKInstance.mu.Unlock()
		return armSDKInstance, nil
	}

	a := &armSDK{
		refs:   1,
		weight: 0,
		stopCh: make(chan struct{}),
	}

	// Initialize all motor commands with mode=1 (enabled) and proper
	// holding gains. Gains are per-motor-type from the official Unitree
	// g1_dual_arm_example.cpp: GearboxL (knee) kp=100, others kp=40,
	// all kd=1. For non-arm motors (legs, torso), the q position is
	// echoed from lowstate each tick in snapshotLocked() so they hold
	// their current position rather than commanding zero.
	for i := range a.commanded {
		a.commanded[i].mode = 1
		a.commanded[i].kd = 1
		a.commanded[i].kp = 40
	}
	// Knees use GearboxL → higher kp.
	a.commanded[G1JointLeftKnee].kp = 100
	a.commanded[G1JointRightKnee].kp = 100

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
func (a *armSDK) rampWeight(target float32, duration time.Duration) error {
	a.mu.Lock()
	start := a.weight
	a.mu.Unlock()

	if target > 0 {
		if !a.hasState.Load() {
			return fmt.Errorf("cannot engage arm_sdk: no lowstate received yet")
		}
		if err := a.ensureWriterLocked(); err != nil {
			return err
		}
	}

	steps := int(duration / (20 * time.Millisecond))
	if steps < 1 {
		steps = 1
	}
	for i := 1; i <= steps; i++ {
		t := float32(i) / float32(steps)
		w := start + (target-start)*t
		a.mu.Lock()
		a.weight = w
		a.mu.Unlock()
		time.Sleep(20 * time.Millisecond)
	}
	return nil
}

// ensureWriterLocked creates the DDS writer if not already created.
// NOT locked — caller must handle locking if needed, or call outside lock.
func (a *armSDK) ensureWriterLocked() error {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.ensureWriter()
}

// setWeight sets the arm_sdk blending weight (0..1). When non-zero, arm_sdk
// motor commands override the sport controller for the arm joints.
// The first call with w > 0 lazily creates the DDS writer on rt/arm_sdk.
// Requires lowstate data to be available so leg positions can be echoed.
func (a *armSDK) setWeight(w float32) error {
	a.mu.Lock()
	defer a.mu.Unlock()
	if w < 0 {
		w = 0
	} else if w > 1 {
		w = 1
	}
	if w > 0 {
		if !a.hasState.Load() {
			return fmt.Errorf("cannot engage arm_sdk: no lowstate received yet")
		}
		if err := a.ensureWriter(); err != nil {
			return err
		}
	}
	a.weight = w
	return nil
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
// LowCmd only when arm_sdk is actively driving the arms (weight > 0) —
// any publishing on rt/arm_sdk, even at weight=0, puts the G1 firmware
// into low-level control mode and makes the sport service refuse all
// RPCs with status=7301. Staying silent when dormant lets sport retain
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
			wasActive := a.weight > 0 && a.writer != 0
			a.weight = 0
			a.commanded[G1ArmSDKWeightIndex].q = 0
			cmd := a.snapshotLocked()
			a.mu.Unlock()
			if wasActive {
				C.unitree_dds_publish_lowcmd(a.writer, &cmd)
			}
			return
		case <-ticker.C:
		}

		a.mu.Lock()
		a.commanded[G1ArmSDKWeightIndex].q = C.float(a.weight)
		active := a.weight > 0
		cmd := a.snapshotLocked()
		a.mu.Unlock()

		if active && !a.paused.Load() && a.writer != 0 {
			C.unitree_dds_publish_lowcmd(a.writer, &cmd)
		}

		var ls C.unitree_hg_lowstate_t
		if rc := C.unitree_dds_take_lowstate(a.reader, 0, &ls); rc == 0 {
			a.mu.Lock()
			a.latestState = ls
			a.lastStateTime = time.Now()
			a.mu.Unlock()
			a.hasState.Store(true)
		}
	}
}

// snapshotLocked builds a LowCmd from the current commanded array. Caller
// must hold a.mu. mode_pr and mode_machine are echoed from the latest
// LowState (required by firmware — sending 0 causes body to drop into
// safety mode). For non-arm motors (legs, torso), current positions are
// echoed from lowstate so they hold steady under the weight blend.
// CRC32 is computed over the entire struct.
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

	// For non-arm motors (legs 0-14, and motors 30-34): echo current
	// position from lowstate so that when weight > 0 the firmware holds
	// these joints at their current position rather than commanding
	// them to q=0 with zero torque (which collapses the legs).
	if a.hasState.Load() {
		for i := 0; i < G1JointLeftShoulderPitch; i++ {
			cmd.motor_cmd[i].q = a.latestState.motor_state[i].q
		}
		for i := G1ArmSDKWeightIndex + 1; i < G1NumMotors; i++ {
			cmd.motor_cmd[i].q = a.latestState.motor_state[i].q
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
