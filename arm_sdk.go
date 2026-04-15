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

	stopCh chan struct{}
	wg     sync.WaitGroup
}

var (
	armSDKMu       sync.Mutex
	armSDKInstance *armSDK
)

// getArmSDK returns the shared arm SDK singleton, creating it on first use.
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

	// Initialize commanded motor cmds with safe defaults (kp=0, kd=0 so
	// nothing moves until a component sets real gains).
	for i := range a.commanded {
		a.commanded[i].mode = 1 // PMSM mode
	}

	cmdTopic := C.CString("rt/arm_sdk")
	defer C.free(unsafe.Pointer(cmdTopic))
	if rc := C.unitree_dds_create_lowcmd_writer(cmdTopic, &a.writer); rc != 0 {
		return nil, fmt.Errorf("create arm_sdk writer failed (rc=%d)", rc)
	}

	stateTopic := C.CString("rt/lowstate")
	defer C.free(unsafe.Pointer(stateTopic))
	if rc := C.unitree_dds_subscribe(stateTopic, 1 /* lowstate */, &a.reader); rc != 0 {
		C.unitree_dds_close_writer(a.writer)
		return nil, fmt.Errorf("subscribe rt/lowstate failed (rc=%d)", rc)
	}

	// Background poller for lowstate (~100 Hz); also handles control-rate
	// publish of the latest commanded LowCmd at the same cadence so the
	// robot keeps tracking even when joint setpoints are static.
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

// setWeight sets the arm_sdk blending weight (0..1). When non-zero, arm_sdk
// motor commands override the sport controller for the arm joints.
func (a *armSDK) setWeight(w float32) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if w < 0 {
		w = 0
	} else if w > 1 {
		w = 1
	}
	a.weight = w
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

// run is the publish/poll loop. Publishes the current LowCmd at ~50 Hz and
// reads the latest LowState in the same iteration.
func (a *armSDK) run() {
	defer a.wg.Done()
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-a.stopCh:
			// Best-effort: zero out the weight before exit so sport mode
			// reclaims the arms.
			a.mu.Lock()
			a.weight = 0
			a.commanded[G1ArmSDKWeightIndex].q = 0
			cmd := a.snapshotLocked()
			a.mu.Unlock()
			C.unitree_dds_publish_lowcmd(a.writer, &cmd)
			return
		case <-ticker.C:
		}

		a.mu.Lock()
		a.commanded[G1ArmSDKWeightIndex].q = C.float(a.weight)
		cmd := a.snapshotLocked()
		a.mu.Unlock()

		C.unitree_dds_publish_lowcmd(a.writer, &cmd)

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
// must hold a.mu.
func (a *armSDK) snapshotLocked() C.unitree_hg_lowcmd_t {
	var cmd C.unitree_hg_lowcmd_t
	for i := 0; i < G1NumMotors; i++ {
		cmd.motor_cmd[i] = a.commanded[i]
	}
	return cmd
}
