package main

import "testing"

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
	a.latestState.motor_state[G1JointWaistPitch].q = 0.4 // the torso sags...
	a.latestState.motor_state[G1JointLeftKnee].q = 0.7
	a.mu.Lock()
	cmd, writer, closeAfter := a.nextFrameLocked()
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
	cmd, writer, closeAfter = a.nextFrameLocked()
	a.mu.Unlock()
	if writer != 1 || !closeAfter || cmd.motor_cmd[G1ArmSDKWeightIndex].q != 0 {
		t.Fatalf("release frame: writer %d, closeAfter %v, weight %v", writer, closeAfter,
			cmd.motor_cmd[G1ArmSDKWeightIndex].q)
	}
	a.mu.Lock()
	_, writer, _ = a.nextFrameLocked()
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
