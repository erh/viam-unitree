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
// (commit e4fa30f). Its meshes are not vendored; g1Boxes stands in for them as collision geometry.
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
		Lower  float64 `xml:"lower,attr"`
		Upper  float64 `xml:"upper,attr"`
		Effort float64 `xml:"effort,attr"` // the motor's rated torque, N·m
	} `xml:"limit"`
}

type urdfRef struct {
	Link string `xml:"link,attr"`
}

// g1Boxes is one collision box per URDF link: center and size in mm, in the link's frame. They are
// the bounds of the upstream g1_description meshes; per link a box beat a covering capsule.
var g1Boxes = map[string][2]r3.Vector{
	"pelvis":                    {{X: 0, Y: 0, Z: -72}, {X: 99, Y: 130, Z: 151}},
	"pelvis_contour_link":       {{X: 2, Y: 0, Z: -90}, {X: 121, Y: 137, Z: 126}},
	"left_hip_pitch_link":       {{X: -1, Y: 49, Z: -21}, {X: 84, Y: 100, Z: 119}},
	"left_hip_roll_link":        {{X: 27, Y: 0, Z: -39}, {X: 108, Y: 99, Z: 172}},
	"left_hip_yaw_link":         {{X: -37, Y: -1, Z: -113}, {X: 181, Y: 90, Z: 228}},
	"left_knee_link":            {{X: 12, Y: 0, Z: -138}, {X: 105, Y: 73, Z: 357}},
	"left_ankle_pitch_link":     {{X: -6, Y: 0, Z: -6}, {X: 38, Y: 15, Z: 39}},
	"left_ankle_roll_link":      {{X: 38, Y: 0, Z: -6}, {X: 210, Y: 77, Z: 60}},
	"right_hip_pitch_link":      {{X: -1, Y: -49, Z: -21}, {X: 84, Y: 100, Z: 119}},
	"right_hip_roll_link":       {{X: 27, Y: 0, Z: -39}, {X: 108, Y: 99, Z: 172}},
	"right_hip_yaw_link":        {{X: -37, Y: 1, Z: -113}, {X: 181, Y: 90, Z: 228}},
	"right_knee_link":           {{X: 12, Y: 0, Z: -138}, {X: 105, Y: 73, Z: 357}},
	"right_ankle_pitch_link":    {{X: -6, Y: 0, Z: -6}, {X: 38, Y: 15, Z: 39}},
	"right_ankle_roll_link":     {{X: 38, Y: 0, Z: -6}, {X: 210, Y: 77, Z: 60}},
	"waist_yaw_link":            {{X: 0, Y: 0, Z: 23}, {X: 86, Y: 86, Z: 48}},
	"waist_roll_link":           {{X: 0, Y: 0, Z: 9}, {X: 22, Y: 24, Z: 40}},
	"torso_link":                {{X: 8, Y: 0, Z: 145}, {X: 152, Y: 217, Z: 312}},
	"logo_link":                 {{X: 2, Y: 0, Z: 272}, {X: 151, Y: 151, Z: 31}},
	"head_link":                 {{X: 4, Y: 0, Z: 428}, {X: 142, Y: 157, Z: 207}},
	"waist_support_link":        {{X: 1, Y: 0, Z: 32}, {X: 118, Y: 136, Z: 43}},
	"left_shoulder_pitch_link":  {{X: 0, Y: 37, Z: -5}, {X: 64, Y: 76, Z: 92}},
	"left_shoulder_roll_link":   {{X: 0, Y: 4, Z: -35}, {X: 83, Y: 74, Z: 138}},
	"left_shoulder_yaw_link":    {{X: 8, Y: -4, Z: -41}, {X: 83, Y: 76, Z: 147}},
	"left_elbow_link":           {{X: 35, Y: 2, Z: -8}, {X: 131, Y: 68, Z: 77}},
	"left_wrist_roll_link":      {{X: 28, Y: 0, Z: 0}, {X: 57, Y: 62, Z: 61}},
	"left_wrist_pitch_link":     {{X: 23, Y: 0, Z: 0}, {X: 105, Y: 58, Z: 58}},
	"left_wrist_yaw_link":       {{X: 12, Y: 3, Z: 0}, {X: 61, Y: 55, Z: 62}},
	"left_rubber_hand":          {{X: 66, Y: -15, Z: 10}, {X: 133, Y: 68, Z: 108}},
	"right_shoulder_pitch_link": {{X: 0, Y: -37, Z: -5}, {X: 64, Y: 76, Z: 92}},
	"right_shoulder_roll_link":  {{X: 0, Y: -4, Z: -35}, {X: 83, Y: 74, Z: 138}},
	"right_shoulder_yaw_link":   {{X: 8, Y: 4, Z: -41}, {X: 83, Y: 76, Z: 147}},
	"right_elbow_link":          {{X: 35, Y: -2, Z: -8}, {X: 131, Y: 68, Z: 77}},
	"right_wrist_roll_link":     {{X: 28, Y: 0, Z: 0}, {X: 57, Y: 62, Z: 61}},
	"right_wrist_pitch_link":    {{X: 23, Y: 0, Z: 0}, {X: 105, Y: 58, Z: 58}},
	"right_wrist_yaw_link":      {{X: 12, Y: -3, Z: 0}, {X: 61, Y: 55, Z: 62}},
	"right_rubber_hand":         {{X: 66, Y: 15, Z: 10}, {X: 133, Y: 68, Z: 108}},
}

// g1Kinematics builds a Viam model from the G1 URDF covering everything below root, which is a
// link, or a joint to start the model at that joint. Subtrees under the skip joints are left out.
// tip is the URDF link whose pose the model reports as its end position.
//
// RDK's URDF parser isn't used because it stores each joint's origin on the parent link, so a link
// with several moving children (pelvis, torso_link) keeps only the last child's offset. Here every
// joint origin gets its own static frame, and every URDF link a zero-offset frame after its joint
// that carries the link's box (RDK places a link's geometry before that link's own offset).
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
	addLink := func(link, parent string) error {
		lc := referenceframe.LinkConfig{ID: link, Parent: parent}
		if b, ok := g1Boxes[link]; ok {
			box, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(b[0]), b[1], link)
			if err != nil {
				return err
			}
			if lc.Geometry, err = spatialmath.NewGeometryConfig(box); err != nil {
				return err
			}
		}
		cfg.Links = append(cfg.Links, lc)
		frameOf[link] = link
		return nil
	}
	if !slices.ContainsFunc(doc.Joints, func(j urdfJoint) bool { return j.Name == root }) {
		if err := addLink(root, ""); err != nil {
			return nil, err
		}
	}
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
			ID:          j.Name + "_origin",
			Parent:      parent,
			Translation: r3.Vector{X: xyz[0] * 1000, Y: xyz[1] * 1000, Z: xyz[2] * 1000},
			Orientation: orient,
		}
		cfg.Links = append(cfg.Links, origin)
		parent = origin.ID
		switch j.Type {
		case referenceframe.FixedJoint:
		case referenceframe.RevoluteJoint:
			if err := parseVec3(j.Axis.XYZ, &axis); err != nil {
				return nil, fmt.Errorf("joint %s axis: %w", j.Name, err)
			}
			cfg.Joints = append(cfg.Joints, referenceframe.JointConfig{
				ID:     j.Name,
				Type:   j.Type,
				Parent: origin.ID,
				Axis:   spatialmath.AxisConfig{X: axis[0], Y: axis[1], Z: axis[2]},
				Min:    utils.RadToDeg(j.Limit.Lower),
				Max:    utils.RadToDeg(j.Limit.Upper),
			})
			parent = j.Name
		default:
			return nil, fmt.Errorf("joint %s: unsupported type %q", j.Name, j.Type)
		}
		if err := addLink(j.Child.Link, parent); err != nil {
			return nil, err
		}
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
