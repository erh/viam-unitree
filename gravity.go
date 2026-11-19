package main

import (
	"encoding/xml"
	"fmt"
	"math"
	"slices"
	"strings"
	"sync"

	"github.com/golang/geo/r3"
)

// Gravity compensation for the upper body arm_sdk drives.
//
// A position hold (kp × error) only pushes back once a joint has moved off its
// target, so weight on a joint shows up as droop: two extended arms (7 kg) lean
// the waist about 15° at kp 60, and the arms sag below their targets. Sending
// the torque that gravity puts on each joint as feedforward (the motor command's
// tau) holds the pose without that error.

// gravityAccel is standard gravity, m/s².
const gravityAccel = 9.81

// gravityTorqueShare is the share of a motor's rated torque (URDF effort) the
// feedforward may use, leaving the rest for the position hold. Gravity alone
// stays well under it: an extended arm needs ~8.6 N·m of a shoulder's 12.5.
const gravityTorqueShare = 0.5

type mat3 [3][3]float64

func identity3() mat3 { return mat3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} }

func (m mat3) mul(n mat3) mat3 {
	var out mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			for k := 0; k < 3; k++ {
				out[i][j] += m[i][k] * n[k][j]
			}
		}
	}
	return out
}

func (m mat3) apply(v r3.Vector) r3.Vector {
	return r3.Vector{
		X: m[0][0]*v.X + m[0][1]*v.Y + m[0][2]*v.Z,
		Y: m[1][0]*v.X + m[1][1]*v.Y + m[1][2]*v.Z,
		Z: m[2][0]*v.X + m[2][1]*v.Y + m[2][2]*v.Z,
	}
}

func (m mat3) transpose() mat3 {
	var out mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			out[i][j] = m[j][i]
		}
	}
	return out
}

// rpyMat is URDF's fixed-axis roll-pitch-yaw rotation, Rz(yaw)·Ry(pitch)·Rx(roll).
func rpyMat(roll, pitch, yaw float64) mat3 {
	sr, cr := math.Sincos(roll)
	sp, cp := math.Sincos(pitch)
	sy, cy := math.Sincos(yaw)
	return mat3{
		{cy * cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
		{sy * cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
		{-sp, cp * sr, cp * cr},
	}
}

// axisAngle is the rotation by angle about a unit axis (Rodrigues' formula).
func axisAngle(axis r3.Vector, angle float64) mat3 {
	s, c := math.Sincos(angle)
	t := 1 - c
	x, y, z := axis.X, axis.Y, axis.Z
	return mat3{
		{t*x*x + c, t*x*y - s*z, t*x*z + s*y},
		{t*x*y + s*z, t*y*y + c, t*y*z - s*x},
		{t*x*z - s*y, t*y*z + s*x, t*z*z + c},
	}
}

// pelvisGravity returns gravity's acceleration in the pelvis frame, given the
// pelvis orientation in the world as a unit quaternion (w, x, y, z), as the G1's
// IMU reports it. An unusable quaternion is taken as upright.
func pelvisGravity(w, x, y, z float64) r3.Vector {
	n := math.Sqrt(w*w + x*x + y*y + z*z)
	if n < 0.5 || math.IsNaN(n) {
		return r3.Vector{Z: -gravityAccel}
	}
	w, x, y, z = w/n, x/n, y/n, z/n
	toWorld := mat3{
		{1 - 2*(y*y+z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)},
		{2 * (x*y + w*z), 1 - 2*(x*x+z*z), 2 * (y*z - w*x)},
		{2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2*(x*x+y*y)},
	}
	return toWorld.transpose().apply(r3.Vector{Z: -gravityAccel})
}

// pelvisTilt returns how far the pelvis is from upright, in radians, given its
// orientation as in pelvisGravity.
func pelvisTilt(w, x, y, z float64) float64 {
	g := pelvisGravity(w, x, y, z)
	return math.Acos(math.Max(-1, math.Min(1, -g.Z/gravityAccel)))
}

// gravityJoint is one URDF joint of the upper body and the link it carries.
type gravityJoint struct {
	name   string
	motor  int // rt/lowstate motor index, or -1 for a fixed joint
	parent int // index of the joint carrying this joint's parent link, or -1 for the pelvis
	origin r3.Vector
	rot    mat3
	axis   r3.Vector
	mass   float64   // the child link's mass, kg
	com    r3.Vector // the child link's center of mass, in its frame, m
}

// gravityModel is the URDF subtree from waist_yaw_joint up (waist, torso, head
// and arms), in pelvis coordinates. Joints are ordered parents first.
type gravityModel struct {
	joints  []gravityJoint
	carries [][]bool // carries[i][k]: joint i moves link k (k == i, or k is above i)
	// effort is each motor's rated torque from the URDF, N·m (all 29 motors).
	effort [G1NumMotors]float64
}

// torqueLimit caps a motor's gravity feedforward, N·m.
func (m *gravityModel) torqueLimit(motor int) float64 {
	return gravityTorqueShare * m.effort[motor]
}

type urdfInertialLink struct {
	Name     string `xml:"name,attr"`
	Inertial *struct {
		Origin urdfXYZ `xml:"origin"`
		Mass   struct {
			Value float64 `xml:"value,attr"`
		} `xml:"mass"`
	} `xml:"inertial"`
}

func newUpperBodyGravityModel() (*gravityModel, error) {
	var doc struct {
		Links  []urdfInertialLink `xml:"link"`
		Joints []urdfJoint        `xml:"joint"`
	}
	if err := xml.Unmarshal(g1URDF, &doc); err != nil {
		return nil, err
	}
	links := map[string]urdfInertialLink{}
	for _, l := range doc.Links {
		links[l.Name] = l
	}

	m := &gravityModel{}
	carrier := map[string]int{} // link name -> index of the joint that carries it
	for _, j := range doc.Joints {
		if j.Type == "revolute" {
			if motor := slices.Index(g1MotorJoints, strings.TrimSuffix(j.Name, "_joint")); motor >= 0 {
				m.effort[motor] = j.Limit.Effort
			}
		}
	}
	for _, j := range doc.Joints { // the URDF lists parents before children
		parent, ok := carrier[j.Parent.Link]
		if j.Name == "waist_yaw_joint" {
			parent, ok = -1, true
		}
		if !ok {
			continue
		}
		var xyz, rpy, axis, com, comRPY [3]float64
		if err := parseVec3(j.Origin.XYZ, &xyz); err != nil {
			return nil, fmt.Errorf("joint %s origin: %w", j.Name, err)
		}
		if err := parseVec3(j.Origin.RPY, &rpy); err != nil {
			return nil, fmt.Errorf("joint %s rpy: %w", j.Name, err)
		}
		gj := gravityJoint{
			name:   j.Name,
			motor:  -1,
			parent: parent,
			origin: r3.Vector{X: xyz[0], Y: xyz[1], Z: xyz[2]},
			rot:    rpyMat(rpy[0], rpy[1], rpy[2]),
		}
		switch j.Type {
		case "fixed":
		case "revolute":
			if err := parseVec3(j.Axis.XYZ, &axis); err != nil {
				return nil, fmt.Errorf("joint %s axis: %w", j.Name, err)
			}
			gj.axis = r3.Vector{X: axis[0], Y: axis[1], Z: axis[2]}.Normalize()
			gj.motor = slices.Index(g1MotorJoints, strings.TrimSuffix(j.Name, "_joint"))
			if gj.motor < 0 {
				return nil, fmt.Errorf("joint %s is not a G1 motor", j.Name)
			}
		default:
			return nil, fmt.Errorf("joint %s: unsupported type %q", j.Name, j.Type)
		}
		if in := links[j.Child.Link].Inertial; in != nil {
			if err := parseVec3(in.Origin.XYZ, &com); err != nil {
				return nil, fmt.Errorf("link %s inertial origin: %w", j.Child.Link, err)
			}
			if err := parseVec3(in.Origin.RPY, &comRPY); err != nil {
				return nil, fmt.Errorf("link %s inertial rpy: %w", j.Child.Link, err)
			}
			// The inertial frame's rotation doesn't move the center of mass.
			gj.mass = in.Mass.Value
			gj.com = r3.Vector{X: com[0], Y: com[1], Z: com[2]}
		}
		carrier[j.Child.Link] = len(m.joints)
		m.joints = append(m.joints, gj)
	}
	if len(m.joints) == 0 {
		return nil, fmt.Errorf("waist_yaw_joint not found in the URDF")
	}

	m.carries = make([][]bool, len(m.joints))
	for i := range m.joints {
		m.carries[i] = make([]bool, len(m.joints))
	}
	for k := range m.joints {
		for i := k; i >= 0; i = m.joints[i].parent {
			m.carries[i][k] = true
		}
	}
	return m, nil
}

// holdingTorques returns, for each upper-body motor, the torque in N·m (along the
// joint's positive direction) that holds the given pose against gravity. q holds
// joint positions by motor index; gravity is gravity's acceleration in the
// pelvis frame. Entries for other motors are zero.
func (m *gravityModel) holdingTorques(q *[G1NumMotors]float64, gravity r3.Vector) [G1NumMotors]float64 {
	pos, axis, com := m.forward(q)
	var tau [G1NumMotors]float64
	for i, j := range m.joints {
		if j.motor < 0 {
			continue
		}
		// Gravity's torque about this joint from every link it carries; the motor
		// must supply the opposite.
		var gravityTorque float64
		for k, carried := range m.carries[i] {
			if carried && m.joints[k].mass > 0 {
				weight := gravity.Mul(m.joints[k].mass)
				gravityTorque += axis[i].Dot(com[k].Sub(pos[i]).Cross(weight))
			}
		}
		tau[j.motor] = -gravityTorque
	}
	return tau
}

// forward returns, in the pelvis frame, each joint's origin (its child link's
// origin) and axis, and each child link's center of mass, for joint positions q.
func (m *gravityModel) forward(q *[G1NumMotors]float64) (pos, axis, com []r3.Vector) {
	n := len(m.joints)
	pos = make([]r3.Vector, n)
	axis = make([]r3.Vector, n)
	com = make([]r3.Vector, n)
	rot := make([]mat3, n) // child link orientation
	for i, j := range m.joints {
		parentRot, parentPos := identity3(), r3.Vector{}
		if j.parent >= 0 {
			parentRot, parentPos = rot[j.parent], pos[j.parent]
		}
		jointRot := parentRot.mul(j.rot)
		pos[i] = parentPos.Add(parentRot.apply(j.origin))
		rot[i] = jointRot
		if j.motor >= 0 {
			axis[i] = jointRot.apply(j.axis)
			rot[i] = jointRot.mul(axisAngle(j.axis, q[j.motor]))
		}
		com[i] = pos[i].Add(rot[i].apply(j.com))
	}
	return pos, axis, com
}

var (
	upperBodyGravityOnce sync.Once
	upperBodyGravity     *gravityModel
	upperBodyGravityErr  error
)

// upperBodyGravityModel returns the gravity model built from the embedded URDF.
func upperBodyGravityModel() (*gravityModel, error) {
	upperBodyGravityOnce.Do(func() {
		upperBodyGravity, upperBodyGravityErr = newUpperBodyGravityModel()
	})
	return upperBodyGravity, upperBodyGravityErr
}
