package main

import (
	"math"
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"
)

func TestGravityModel(t *testing.T) {
	m, err := upperBodyGravityModel()
	if err != nil {
		t.Fatal(err)
	}

	// The model covers the 17 upper-body motors and the 16.9 kg the URDF puts above the waist.
	motors := map[int]bool{}
	var mass float64
	for _, j := range m.joints {
		if j.motor >= 0 {
			motors[j.motor] = true
		}
		mass += j.mass
	}
	for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
		if !motors[i] {
			t.Errorf("motor %s missing", g1MotorJoints[i])
		}
	}
	if len(motors) != 17 || math.Abs(mass-16.93) > 0.05 {
		t.Errorf("%d motors, %.2f kg; want 17 motors, 16.93 kg", len(motors), mass)
	}

	upright := pelvisGravity(1, 0, 0, 0)
	var zero [G1NumMotors]float64
	// In the URDF's zero pose the elbows are bent 90° with the forearms forward; elbow +90°
	// straightens the arm so it hangs below the shoulder.
	hanging := zero
	hanging[G1JointLeftElbow], hanging[G1JointRightElbow] = math.Pi/2, math.Pi/2
	rest := m.holdingTorques(&hanging, upright)
	// With the pelvis upright, gravity has no torque about the vertical waist yaw axis.
	if math.Abs(rest[G1JointWaistYaw]) > 1e-9 {
		t.Errorf("waist yaw %.3g N·m at rest, want 0", rest[G1JointWaistYaw])
	}
	// Straight arms hang under the shoulders, which carry little, the same on both sides.
	for _, i := range []int{G1JointLeftShoulderPitch, G1JointRightShoulderPitch} {
		if math.Abs(rest[i]) > 1.5 {
			t.Errorf("%s %.2f N·m with the arm hanging, want near 0", g1MotorJoints[i], rest[i])
		}
	}
	if d := rest[G1JointLeftShoulderPitch] - rest[G1JointRightShoulderPitch]; math.Abs(d) > 1e-3 {
		t.Errorf("shoulder pitch torques differ by %.3g between sides", d)
	}

	// Both arms straight forward (shoulder pitch -90°): each shoulder lifts its arm (3.5 kg, center
	// of mass ~0.2 m out), and the waist holds the torso back against the arms' pull forward. Both
	// torques are negative, the direction that raises the arms and leans the torso back.
	forward := hanging
	forward[G1JointLeftShoulderPitch], forward[G1JointRightShoulderPitch] = -math.Pi/2, -math.Pi/2
	f := m.holdingTorques(&forward, upright)
	for _, i := range []int{G1JointLeftShoulderPitch, G1JointRightShoulderPitch} {
		if f[i] > -4 || f[i] < -12 {
			t.Errorf("%s %.2f N·m with the arm forward, want -4 to -12", g1MotorJoints[i], f[i])
		}
	}
	if d := f[G1JointWaistPitch] - rest[G1JointWaistPitch]; d > -10 || d < -25 {
		t.Errorf("waist pitch changes by %.2f N·m with the arms forward, want -10 to -25", d)
	}

	// Pelvis pitched 10° forward: gravity leans forward in the pelvis frame, and the waist must
	// hold the torso back harder than when upright.
	half := 10 * math.Pi / 180 / 2
	tilted := pelvisGravity(math.Cos(half), 0, math.Sin(half), 0)
	if tilted.X <= 0 || math.Abs(tilted.Norm()-gravityAccel) > 1e-9 {
		t.Errorf("gravity %v with the pelvis pitched forward, want +x and |g| = %v", tilted, gravityAccel)
	}
	if lean := m.holdingTorques(&hanging, tilted); lean[G1JointWaistPitch] >= rest[G1JointWaistPitch]-1 {
		t.Errorf("waist pitch %.2f N·m pitched forward vs %.2f upright, want at least 1 N·m more negative",
			lean[G1JointWaistPitch], rest[G1JointWaistPitch])
	}
	if g := pelvisGravity(0, 0, 0, 0); g != upright {
		t.Errorf("zero quaternion gives %v, want upright", g)
	}
}

// TestGravityTorquesMatchEnergy checks the torques against an independent computation: the
// torque that holds a joint is the rate at which the robot's potential energy changes as the
// joint moves (dU/dq), taken numerically from forward kinematics.
func TestGravityTorquesMatchEnergy(t *testing.T) {
	m, err := upperBodyGravityModel()
	if err != nil {
		t.Fatal(err)
	}
	energy := func(q *[G1NumMotors]float64, gravity r3.Vector) float64 {
		_, _, com := m.forward(q)
		var u float64
		for k, j := range m.joints {
			u -= j.mass * gravity.Dot(com[k])
		}
		return u
	}
	rng := rand.New(rand.NewSource(1))
	for trial := 0; trial < 20; trial++ {
		var q [G1NumMotors]float64
		for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
			q[i] = rng.Float64()*2 - 1 // ±57°
		}
		w, x, y := 1.0, rng.Float64()*0.2-0.1, rng.Float64()*0.2-0.1 // tilted up to ~11°
		gravity := pelvisGravity(w, x, y, 0)
		tau := m.holdingTorques(&q, gravity)
		const h = 1e-6
		for i := G1JointWaistYaw; i <= G1JointRightWristYaw; i++ {
			plus, minus := q, q
			plus[i] += h
			minus[i] -= h
			want := (energy(&plus, gravity) - energy(&minus, gravity)) / (2 * h)
			if math.Abs(tau[i]-want) > 1e-5 {
				t.Fatalf("trial %d, %s: %.6f N·m, energy gradient %.6f", trial, g1MotorJoints[i], tau[i], want)
			}
		}
	}
}

// TestArmSDKGravityFeedforward checks the published frame carries the scaled, capped torques.
func TestArmSDKGravityFeedforward(t *testing.T) {
	model, err := upperBodyGravityModel()
	if err != nil {
		t.Fatal(err)
	}
	a := &armSDK{gravityScale: 1}
	a.initCommands()
	a.latestState.imu_state.quaternion[0] = 1
	a.latestState.motor_state[G1JointLeftShoulderPitch].q = -1.5708 // straight arm forward
	a.latestState.motor_state[G1JointLeftElbow].q = 1.5708
	a.hasState.Store(true)

	a.mu.Lock()
	tau, ok := a.gravityTorquesLocked()
	cmd := a.snapshotLocked()
	a.mu.Unlock()
	if !ok || tau[G1JointLeftShoulderPitch] > -4 {
		t.Fatalf("left shoulder pitch feedforward %.2f N·m (ok %v), want below -4", tau[G1JointLeftShoulderPitch], ok)
	}
	for i := 0; i < G1NumMotors; i++ {
		got := float64(cmd.motor_cmd[i].tau)
		upper := i >= G1JointWaistYaw && i <= G1JointRightWristYaw
		switch {
		case upper && math.Abs(got-tau[i]) > 1e-4:
			t.Errorf("%s tau %.4f in the frame, want %.4f", g1MotorJoints[i], got, tau[i])
		case upper && math.Abs(got) > model.torqueLimit(i)+1e-6:
			t.Errorf("%s tau %.2f exceeds its %.1f N·m cap", g1MotorJoints[i], got, model.torqueLimit(i))
		case !upper && got != 0:
			t.Errorf("motor %d tau %.4f, want 0 outside the upper body", i, got)
		}
	}

	// Half scale halves it; 0 turns it off; out-of-range scales are refused.
	if err := a.setGravityScale(0.5); err != nil {
		t.Fatal(err)
	}
	a.mu.Lock()
	half, _ := a.gravityTorquesLocked()
	a.mu.Unlock()
	if math.Abs(half[G1JointLeftShoulderPitch]-tau[G1JointLeftShoulderPitch]/2) > 1e-9 {
		t.Errorf("half scale %.3f, want %.3f", half[G1JointLeftShoulderPitch], tau[G1JointLeftShoulderPitch]/2)
	}
	if err := a.setGravityScale(0); err != nil {
		t.Fatal(err)
	}
	a.mu.Lock()
	cmd = a.snapshotLocked()
	a.mu.Unlock()
	if cmd.motor_cmd[G1JointLeftShoulderPitch].tau != 0 {
		t.Errorf("tau %v with compensation off", cmd.motor_cmd[G1JointLeftShoulderPitch].tau)
	}
	for _, bad := range []float64{-0.1, 2, math.NaN()} {
		if a.setGravityScale(bad) == nil {
			t.Errorf("scale %v accepted", bad)
		}
	}
}
