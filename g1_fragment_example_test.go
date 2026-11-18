package main

import (
	"encoding/json"
	"os"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// TestFragmentExample builds the frame system from g1_fragment_example.json with this module's
// models, at zero joint angles, and checks the pieces line up.
func TestFragmentExample(t *testing.T) {
	data, err := os.ReadFile("g1_fragment_example.json")
	if err != nil {
		t.Fatal(err)
	}
	var frag struct {
		Components []struct {
			Name  string
			Model string
			Frame *referenceframe.LinkConfig
		}
	}
	if err := json.Unmarshal(data, &frag); err != nil {
		t.Fatal(err)
	}
	kinematics := map[string]func(string) (referenceframe.Model, error){
		"erh:viam-unitree:g1-body":      g1BodyKinematics,
		"erh:viam-unitree:g1-left-arm":  g1LeftArmKinematics,
		"erh:viam-unitree:g1-right-arm": g1RightArmKinematics,
	}
	var parts []*referenceframe.FrameSystemPart
	for _, c := range frag.Components {
		if c.Frame == nil {
			continue
		}
		c.Frame.ID = c.Name
		lif, err := c.Frame.ParseConfig()
		if err != nil {
			t.Fatal(err)
		}
		part := &referenceframe.FrameSystemPart{FrameConfig: lif}
		if build, ok := kinematics[c.Model]; ok {
			if part.ModelFrame, err = build(c.Name); err != nil {
				t.Fatal(err)
			}
		}
		parts = append(parts, part)
	}
	fs, err := referenceframe.NewFrameSystem("g1", parts, nil)
	if err != nil {
		t.Fatal(err)
	}
	in := referenceframe.NewZeroLinearInputs(fs)
	inWorld := func(frame string, p r3.Vector) r3.Vector {
		t.Helper()
		tf, err := fs.Transform(in, referenceframe.NewPoseInFrame(frame, spatialmath.NewPoseFromPoint(p)), referenceframe.World)
		if err != nil {
			t.Fatal(err)
		}
		return tf.(*referenceframe.PoseInFrame).Pose().Point()
	}

	// base -> pelvis (z 793) -> torso_link (-3.96, 0, 54) -> palm (245.24, 151.65, 41.23)
	near(t, "left hand", inWorld("left-arm", r3.Vector{}), r3.Vector{X: 241.28, Y: 151.65, Z: 888.23})
	// The camera sits on the head, looks forward and 47.6 degrees down, and image right is robot right.
	cam := inWorld("camera", r3.Vector{})
	near(t, "camera", cam, r3.Vector{X: 53.66, Y: 17.53, Z: 1266.87})
	near(t, "camera lens axis", inWorld("camera", r3.Vector{Z: 1000}).Sub(cam), r3.Vector{X: 674.3, Z: -738.5})
	near(t, "camera image right", inWorld("camera", r3.Vector{X: 1000}).Sub(cam), r3.Vector{Y: -1000})

	// With straight legs the 60 mm tall foot boxes stand on z = 0.
	geoms, err := referenceframe.FrameSystemGeometriesLinearInputs(fs, in)
	if err != nil {
		t.Fatal(err)
	}
	var foot r3.Vector
	for _, g := range geoms["body"].Geometries() {
		if g.Label() == "body:left_ankle_roll_link" {
			foot = g.Pose().Point()
		}
	}
	near(t, "left foot box", foot, r3.Vector{X: 38, Y: 118.51, Z: 30.14})
}
