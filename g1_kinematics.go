package main

import (
	_ "embed"
	"encoding/xml"
	"fmt"
	"slices"
	"strings"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

// g1_29dof.urdf is vendored unmodified from https://github.com/isri-aist/g1_description
// (commit e4fa30f). Meshes are not vendored; the models below carry kinematics only.
//
//go:embed g1_29dof.urdf
var g1URDF []byte

// g1MotorJoints holds the URDF joint names (without "_joint") in rt/lowstate motor order.
var g1MotorJoints = []string{
	"left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
	"right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
	"waist_yaw", "waist_roll", "waist_pitch",
	"left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
	"left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
	"right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
	"right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
}

type urdfXYZ struct {
	XYZ string `xml:"xyz,attr"`
	RPY string `xml:"rpy,attr"`
}

type urdfJoint struct {
	Name   string  `xml:"name,attr"`
	Type   string  `xml:"type,attr"`
	Parent urdfRef `xml:"parent"`
	Child  urdfRef `xml:"child"`
	Origin urdfXYZ `xml:"origin"`
	Axis   urdfXYZ `xml:"axis"`
	Limit  struct {
		Lower float64 `xml:"lower,attr"`
		Upper float64 `xml:"upper,attr"`
	} `xml:"limit"`
}

type urdfRef struct {
	Link string `xml:"link,attr"`
}

// g1Kinematics builds a Viam model from the G1 URDF covering everything below root, which is a
// link, or a joint to start the model at that joint. Subtrees under the skip joints are left out.
// tip is the URDF link whose pose the model reports as its end position.
//
// RDK's URDF parser isn't used because it stores each joint's origin on the parent link, so a link
// with several moving children (pelvis, torso_link) keeps only the last child's offset. Here every
// joint origin gets its own static frame instead.
func g1Kinematics(name, root, tip string, skip ...string) (referenceframe.Model, error) {
	var doc struct {
		Joints []urdfJoint `xml:"joint"`
	}
	if err := xml.Unmarshal(g1URDF, &doc); err != nil {
		return nil, err
	}

	// frameOf maps a URDF link to the model frame whose output is that link; "" is the model base.
	frameOf := map[string]string{root: ""}
	cfg := &referenceframe.ModelConfigJSON{Name: name, KinParamType: "SVA"}
	for _, j := range doc.Joints { // the URDF lists parents before children
		parent, ok := frameOf[j.Parent.Link]
		if j.Name == root {
			parent, ok = "", true
		}
		if !ok || slices.Contains(skip, j.Name) {
			continue
		}
		var xyz, rpy, axis [3]float64
		if err := parseVec3(j.Origin.XYZ, &xyz); err != nil {
			return nil, fmt.Errorf("joint %s origin: %w", j.Name, err)
		}
		if err := parseVec3(j.Origin.RPY, &rpy); err != nil {
			return nil, fmt.Errorf("joint %s rpy: %w", j.Name, err)
		}
		orient, err := spatialmath.NewOrientationConfig(&spatialmath.EulerAngles{Roll: rpy[0], Pitch: rpy[1], Yaw: rpy[2]})
		if err != nil {
			return nil, err
		}
		origin := referenceframe.LinkConfig{
			ID:          j.Child.Link,
			Parent:      parent,
			Translation: r3.Vector{X: xyz[0] * 1000, Y: xyz[1] * 1000, Z: xyz[2] * 1000},
			Orientation: orient,
		}
		if j.Type == referenceframe.FixedJoint {
			cfg.Links = append(cfg.Links, origin)
			frameOf[j.Child.Link] = j.Child.Link
			continue
		}
		if j.Type != referenceframe.RevoluteJoint {
			return nil, fmt.Errorf("joint %s: unsupported type %q", j.Name, j.Type)
		}
		if err := parseVec3(j.Axis.XYZ, &axis); err != nil {
			return nil, fmt.Errorf("joint %s axis: %w", j.Name, err)
		}
		origin.ID = j.Name + "_origin"
		cfg.Links = append(cfg.Links, origin)
		cfg.Joints = append(cfg.Joints, referenceframe.JointConfig{
			ID:     j.Name,
			Type:   j.Type,
			Parent: origin.ID,
			Axis:   spatialmath.AxisConfig{X: axis[0], Y: axis[1], Z: axis[2]},
			Min:    utils.RadToDeg(j.Limit.Lower),
			Max:    utils.RadToDeg(j.Limit.Upper),
		})
		frameOf[j.Child.Link] = j.Name
	}

	tipFrame, ok := frameOf[tip]
	if !ok {
		return nil, fmt.Errorf("tip %q is not below root %q", tip, root)
	}
	cfg.OutputFrames = []string{tipFrame}
	return cfg.ParseConfig(name)
}

// parseVec3 reads a URDF "x y z" attribute; a missing attribute is all zeros, as URDF specifies.
func parseVec3(s string, v *[3]float64) error {
	if s == "" {
		return nil
	}
	_, err := fmt.Sscan(s, &v[0], &v[1], &v[2])
	return err
}

// g1JointIndices maps each joint of a model, in input order, to its rt/lowstate motor index.
// Joint names may carry a "_joint" suffix (URDF style) or not (the old JSON models).
func g1JointIndices(model referenceframe.Model) ([]int, error) {
	sm, ok := model.(*referenceframe.SimpleModel)
	if !ok {
		return nil, fmt.Errorf("model %T has no joint names", model)
	}
	names := sm.MoveableFrameNames()
	indices := make([]int, len(names))
	for i, n := range names {
		indices[i] = slices.Index(g1MotorJoints, strings.TrimSuffix(n, "_joint"))
		if indices[i] < 0 {
			return nil, fmt.Errorf("joint %q is not a G1 motor", n)
		}
	}
	return indices, nil
}

func g1LeftArmKinematics(name string) (referenceframe.Model, error) {
	return g1Kinematics(name, "left_shoulder_pitch_joint", "left_rubber_hand")
}

func g1RightArmKinematics(name string) (referenceframe.Model, error) {
	return g1Kinematics(name, "right_shoulder_pitch_joint", "right_rubber_hand")
}

// g1BodyKinematics is pelvis, legs and waist up to torso_link; the arms attach there as their own
// components (frame parent = the body).
func g1BodyKinematics(name string) (referenceframe.Model, error) {
	return g1Kinematics(name, "pelvis", "torso_link", "left_shoulder_pitch_joint", "right_shoulder_pitch_joint")
}
