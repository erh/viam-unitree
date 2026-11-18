package main

import (
	"math"
	"slices"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

func zeroPose(t *testing.T, m referenceframe.Model) spatialmath.Pose {
	t.Helper()
	pose, err := m.Transform(make([]referenceframe.Input, len(m.DoF())))
	if err != nil {
		t.Fatal(err)
	}
	return pose
}

func near(t *testing.T, what string, got, want r3.Vector) {
	t.Helper()
	if got.Sub(want).Norm() > 0.5 {
		t.Errorf("%s: got %v, want %v (mm)", what, got, want)
	}
}

func TestG1Kinematics(t *testing.T) {
	body, err := g1BodyKinematics("body")
	if err != nil {
		t.Fatal(err)
	}
	left, err := g1LeftArmKinematics("left")
	if err != nil {
		t.Fatal(err)
	}
	right, err := g1RightArmKinematics("right")
	if err != nil {
		t.Fatal(err)
	}

	// Every model joint maps to a motor; together they cover all 29 exactly once.
	var all []int
	for _, m := range []referenceframe.Model{body, left, right} {
		idx, err := g1JointIndices(m)
		if err != nil {
			t.Fatal(err)
		}
		all = append(all, idx...)
	}
	slices.Sort(all)
	for i, v := range all {
		if v != i {
			t.Fatalf("motor indices %v, want 0..28 once each", all)
		}
	}
	if idx, _ := g1JointIndices(left); idx[0] != G1JointLeftShoulderPitch || idx[6] != G1JointLeftWristYaw {
		t.Errorf("left arm indices %v not shoulder-to-wrist", idx)
	}

	// pelvis -> torso_link: waist_roll origin + waist_pitch origin.
	near(t, "torso", zeroPose(t, body).Point(), r3.Vector{X: -3.9635, Y: 0, Z: 54})

	// torso_link -> palm, summed by hand from the URDF (shoulder_pitch rolls 0.27931 rad, so
	// shoulder_roll's (0, 38, -13.831) offset lands at (0, 40.34, -2.82)).
	near(t, "left hand", zeroPose(t, left).Point(), r3.Vector{X: 245.24, Y: 151.69, Z: 41.24})
	near(t, "right hand", zeroPose(t, right).Point(), r3.Vector{X: 245.24, Y: -151.68, Z: 41.24})

	// The legs branch off pelvis next to the waist; RDK's URDF parser would put both hips at the
	// last child's origin.
	lFoot, err := g1Kinematics("l", "pelvis", "left_ankle_roll_link")
	if err != nil {
		t.Fatal(err)
	}
	rFoot, err := g1Kinematics("r", "pelvis", "right_ankle_roll_link")
	if err != nil {
		t.Fatal(err)
	}
	lp, rp := zeroPose(t, lFoot).Point(), zeroPose(t, rFoot).Point()
	if lp.Y < 100 || math.Abs(lp.Y+rp.Y) > 0.5 || math.Abs(lp.Z-rp.Z) > 0.5 {
		t.Errorf("feet not mirrored: left %v right %v", lp, rp)
	}

	// The model survives the trip to viam-server (GetKinematics) unchanged.
	back, err := referenceframe.KinematicModelFromProtobuf("body", referenceframe.KinematicModelToProtobuf(body))
	if err != nil {
		t.Fatal(err)
	}
	if !spatialmath.PoseAlmostEqual(zeroPose(t, back), zeroPose(t, body)) || len(back.DoF()) != 15 {
		t.Errorf("round trip changed the model")
	}
}
