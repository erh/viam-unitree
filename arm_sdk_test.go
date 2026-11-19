package main

import (
	"math"
	"strings"
	"testing"
	"time"
)

// TestArmSDKHandoff checks what arm_sdk publishes without opening DDS: a nonzero
// writer handle stands in for the real writer, which ensureWriter then reuses.
func TestArmSDKHandoff(t *testing.T) {
	a := &armSDK{}
	a.initCommands()
	for i := 0; i < G1NumMotors; i++ {
		a.latestState.motor_state[i].q = 0.25
	}
	a.hasState.Store(true)
	a.writer = 1

	// Engaging holds the measured upper-body pose with fixed targets.
	if err := a.setWeight(1); err != nil {
		t.Fatal(err)
	}
	a.latestState.motor_state[G1JointWaistPitch].q = 0.35 // the torso sags (within the waist's yield)...
	a.latestState.motor_state[G1JointLeftKnee].q = 0.7
	a.mu.Lock()
	cmd, writer, closeAfter, _ := a.nextFrameLocked()
	a.mu.Unlock()
	if writer != 1 || closeAfter {
		t.Fatalf("engaged frame: writer %d, closeAfter %v", writer, closeAfter)
	}
	for i := 0; i < G1NumMotors; i++ {
		m := cmd.motor_cmd[i]
		switch {
		case i == G1ArmSDKWeightIndex:
			if m.q != 1 {
				t.Errorf("weight %v, want 1", m.q)
			}
		case i >= G1JointWaistYaw && i <= G1JointRightWristYaw:
			// ...but its target stays where it engaged, so kp pulls it back.
			if m.q != 0.25 || m.mode != 1 || m.kp != upperBodyKp || m.kd != upperBodyKd {
				t.Errorf("upper-body motor %d: q %v mode %v kp %v kd %v", i, m.q, m.mode, m.kp, m.kd)
			}
		default:
			// Legs and 30-34 stay uncommanded, for the balance controller.
			if m.q != 0 || m.mode != 0 || m.kp != 0 || m.kd != 0 {
				t.Errorf("motor %d is commanded: q %v mode %v kp %v kd %v", i, m.q, m.mode, m.kp, m.kd)
			}
		}
	}

	// An arm move sets its targets; releasing sends one zero-weight frame, then closes.
	if err := a.setArmCommand([]int{G1JointLeftElbow}, []float32{1.1}, []float32{60}, []float32{1.5}); err != nil {
		t.Fatal(err)
	}
	if err := a.setWeight(0); err != nil {
		t.Fatal(err)
	}
	a.mu.Lock()
	cmd, writer, closeAfter, _ = a.nextFrameLocked()
	a.mu.Unlock()
	if writer != 1 || !closeAfter || cmd.motor_cmd[G1ArmSDKWeightIndex].q != 0 {
		t.Fatalf("release frame: writer %d, closeAfter %v, weight %v", writer, closeAfter,
			cmd.motor_cmd[G1ArmSDKWeightIndex].q)
	}
	a.mu.Lock()
	_, writer, _, _ = a.nextFrameLocked()
	released := a.writer == 0
	a.mu.Unlock()
	if writer != 0 || !released {
		t.Fatalf("after release: publishing on %d, writer cleared %v", writer, released)
	}
	if armSDKEngaged() {
		t.Error("armSDKEngaged with no singleton")
	}

	// Engaging again (say after a gesture moved the arm) starts from the measured
	// pose, not the old elbow target.
	a.latestState.motor_state[G1JointLeftElbow].q = 0.5
	a.writer = 1
	if err := a.setWeight(1); err != nil {
		t.Fatal(err)
	}
	if q := a.commanded[G1JointLeftElbow].q; q != 0.5 {
		t.Errorf("re-engaged elbow target %v, want measured 0.5", q)
	}

	// Engaging needs lowstate, to know the pose to hold.
	b := &armSDK{}
	b.initCommands()
	if err := b.setWeight(1); err == nil {
		t.Error("engaged without lowstate")
	}
}

// TestMotorDiagnostics checks motor error-code change reports and the motor_status report.
func TestMotorDiagnostics(t *testing.T) {
	a := &armSDK{gravityScale: 1}
	a.initCommands()
	now := time.Now()

	// The first lowstate reports every nonzero code; an unchanged code reports nothing.
	a.latestState.motor_state[G1JointWaistPitch].motorstate = 0x4
	ev := a.motorCodeEventsLocked(now)
	if len(ev) != 1 || !strings.Contains(ev[0], "waist_pitch") || !strings.Contains(ev[0], "0x0 -> 0x4") {
		t.Fatalf("first lowstate: %q", ev)
	}
	if ev := a.motorCodeEventsLocked(now.Add(2 * time.Second)); len(ev) != 0 {
		t.Errorf("unchanged codes reported: %q", ev)
	}
	// A motor reports at most once a second; other motors are unaffected.
	a.latestState.motor_state[G1JointWaistPitch].motorstate = 0
	a.latestState.motor_state[G1JointLeftKnee].motorstate = 0x10
	ev = a.motorCodeEventsLocked(now.Add(500 * time.Millisecond))
	if len(ev) != 1 || !strings.Contains(ev[0], "left_knee") {
		t.Errorf("within a second of the waist report: %q, want only the knee", ev)
	}

	// motor_status reads through the shared lowstate reader.
	armSDKMu.Lock()
	armSDKInstance = a
	armSDKMu.Unlock()
	defer func() {
		armSDKMu.Lock()
		armSDKInstance = nil
		armSDKMu.Unlock()
	}()
	if _, err := motorStatusReport(); err == nil {
		t.Error("motor_status before any lowstate succeeded")
	}
	a.hasState.Store(true)
	a.lastStateTime = now
	r, err := motorStatusReport()
	if err != nil {
		t.Fatal(err)
	}
	knee := r["motors"].(map[string]interface{})["left_knee"].(map[string]interface{})
	faults := r["motors_with_code"].([]interface{})
	if knee["error_code"] != "0x10" || len(faults) != 1 || faults[0] != "left_knee" {
		t.Errorf("knee %v, motors with codes %v", knee, faults)
	}
}

// tiltQuaternion sets the IMU quaternion (w, x, y, z) to the pelvis rolled by deg degrees.
// The values are literals because test files can't convert to cgo types.
func tiltQuaternion(a *armSDK, deg int) {
	q := &a.latestState.imu_state.quaternion
	switch deg { // cos and sin of half the roll
	case 5:
		q[0], q[1] = 0.9990482216, 0.0436193874
	case 20:
		q[0], q[1] = 0.9848077530, 0.1736481777
	case 25:
		q[0], q[1] = 0.9762960071, 0.2164396139
	default:
		panic("no quaternion for that tilt")
	}
	q[2], q[3] = 0, 0
}

// TestArmSDKSafety checks the limits that keep arm_sdk from overloading the upper body.
func TestArmSDKSafety(t *testing.T) {
	model, err := upperBodyGravityModel()
	if err != nil {
		t.Fatal(err)
	}
	// Gravity feedforward is capped at half each motor's rated torque.
	for motor, want := range map[int]float64{G1JointWaistRoll: 17.5, G1JointWaistYaw: 44, G1JointLeftShoulderPitch: 12.5, G1JointLeftWristPitch: 2.5} {
		if got := model.torqueLimit(motor); got != want {
			t.Errorf("%s cap %.1f N·m, want %.1f", g1MotorJoints[motor], got, want)
		}
	}

	a := &armSDK{gravityScale: 1}
	a.initCommands()
	a.hasState.Store(true)
	a.writer = 1

	// No engaging while the pelvis is tipped 20° (falling, or hanging in a harness).
	tiltQuaternion(a, 20)
	if err := a.setWeight(1); err == nil || !strings.Contains(err.Error(), "tilted 20") {
		t.Fatalf("engaged while tilted: %v", err)
	}
	tiltQuaternion(a, 5)
	if err := a.setWeight(1); err != nil {
		t.Fatalf("upright engage: %v", err)
	}

	// A waist forced well off its hold: the hold follows to within waistYield.
	// Arms don't yield; they go where they're commanded.
	a.latestState.motor_state[G1JointWaistRoll].q = 0.4
	a.commanded[G1JointLeftElbow].q = 1
	a.mu.Lock()
	_, _, _, events := a.nextFrameLocked()
	a.mu.Unlock()
	if q := float64(a.commanded[G1JointWaistRoll].q); math.Abs(q-(0.4-waistYield)) > 1e-6 {
		t.Errorf("waist roll hold %.4f, want %.4f", q, 0.4-waistYield)
	}
	if q := float64(a.commanded[G1JointLeftElbow].q); q != 1 {
		t.Errorf("elbow target moved to %.3f", q)
	}
	if !containsAll(events, "waist_roll pushed") {
		t.Errorf("events %q, want a waist_roll yield", events)
	}

	// A command near a motor's rating is flagged: the wrist pitch (5 N·m) held 0.5 rad off at kp 60.
	a.commanded[G1JointLeftWristPitch].q = 0.5
	a.mu.Lock()
	_, _, _, events = a.nextFrameLocked()
	a.mu.Unlock()
	if !containsAll(events, "left_wrist_pitch commanded") {
		t.Errorf("events %q, want a left_wrist_pitch torque warning", events)
	}

	// Tipping past 15° while engaged fades arm_sdk out in 0.2 s, then closes the writer.
	tiltQuaternion(a, 25)
	var closed bool
	for tick := 0; tick < 20 && !closed; tick++ {
		a.mu.Lock()
		cmd, _, c, ev := a.nextFrameLocked()
		a.mu.Unlock()
		closed, events = c, ev
		if tick == 0 && (a.target != 0 || !containsAll(events, "pelvis tilted 25")) {
			t.Fatalf("first tilted frame: target %v, events %q", a.target, events)
		}
		if closed && float64(cmd.motor_cmd[G1ArmSDKWeightIndex].q) != 0 {
			t.Errorf("final frame weight %v", cmd.motor_cmd[G1ArmSDKWeightIndex].q)
		}
		if closed && tick+1 != 10 {
			t.Errorf("released on frame %d, want the 10th (0.2 s)", tick+1)
		}
	}
	if !closed || a.writer != 0 || a.safetyRelease {
		t.Errorf("after tilting: closed %v, writer %d, safetyRelease %v", closed, a.writer, a.safetyRelease)
	}
}

func containsAll(events []string, substr string) bool {
	for _, e := range events {
		if strings.Contains(e, substr) {
			return true
		}
	}
	return false
}
