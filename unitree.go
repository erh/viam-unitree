package main

/*
#cgo CFLAGS: -I${SRCDIR}/capi
#cgo CFLAGS: -I${SRCDIR}/build/_deps/cyclonedds-src/src/core/ddsc/include
#cgo CFLAGS: -I${SRCDIR}/build/_deps/cyclonedds-src/src/ddsrt/include
#cgo CFLAGS: -I${SRCDIR}/build/_deps/cyclonedds-build/src/core/include
#cgo CFLAGS: -I${SRCDIR}/build/_deps/cyclonedds-build/src/ddsrt/include
#cgo LDFLAGS: -L${SRCDIR}/build -ldds_unitree
#cgo LDFLAGS: -L${SRCDIR}/build/lib -lddsc
#cgo LDFLAGS: -lm -lpthread
#cgo LDFLAGS: -Wl,-rpath,${SRCDIR}/build/lib

#include "dds_unitree.h"
#include <stdlib.h>
*/
import "C"

import (
	"encoding/json"
	"fmt"
	"sync"
	"sync/atomic"
	"unsafe"
)

// Unitree G1 sport (loco) API IDs.
// See unitree_sdk2/include/unitree/robot/g1/loco/g1_loco_api.hpp
const (
	ApiLocoGetFsmID       int64 = 7001
	ApiLocoGetFsmMode     int64 = 7002
	ApiLocoGetBalanceMode int64 = 7003
	ApiLocoGetSwingHeight int64 = 7004
	ApiLocoGetStandHeight int64 = 7005
	ApiLocoSetFsmID       int64 = 7101
	ApiLocoSetBalanceMode int64 = 7102
	ApiLocoSetSwingHeight int64 = 7103
	ApiLocoSetStandHeight int64 = 7104
	ApiLocoSetVelocity    int64 = 7105
	ApiLocoSetArmTask     int64 = 7106
	ApiLocoSetSpeedMode   int64 = 7107
)

// G1 FSM (Finite State Machine) IDs used with SetFsmId.
//
// Note: the G1 firmware has evolved the FSM ID space. The older unitree_sdk2
// C++ client used FsmStandUp=4 and FsmStart=500; current firmware (and the
// unitree_sdk2_python client) uses distinct transitional IDs: 706 for
// Squat→StandUp and 702 for Lie→StandUp, plus 200 for Start. Sending the
// legacy values succeeds at the RPC layer (rc=0) but silently does not
// transition state on recent firmware — this is why earlier ready_to_move
// sequences appeared to succeed but left the robot unresponsive to Move.
//
// The canonical Python example (g1_loco_client_example.py option 1) is
// Damp → Squat2StandUp, after which Move commands work immediately — no
// explicit Start(200) is required on current firmware.
const (
	FsmZeroTorque    = 0
	FsmDamp          = 1
	FsmSquat         = 2
	FsmSit           = 3
	FsmStandUp       = 4 // legacy C++ SDK value; current firmware uses 706/702
	FsmStart         = 200
	FsmLie2StandUp   = 702
	FsmSquat2StandUp = 706
)

// Unitree video API IDs.
const (
	ApiGetImageSample int64 = 1001
)

// The DDS participant is process-global and shared by all components.
// We refcount it so the participant stays alive while any component uses
// it, and is cleanly torn down (notifying the robot) when the last
// component closes.
var (
	ddsMu   sync.Mutex
	ddsRefs int
)

// InitDDS initializes (or reuses) the global DDS participant.
// Each call must be paired with a ShutdownDDS().
func InitDDS(domainID int, networkInterface string) error {
	ddsMu.Lock()
	defer ddsMu.Unlock()

	if ddsRefs == 0 {
		cIface := C.CString(networkInterface)
		defer C.free(unsafe.Pointer(cIface))
		rc := C.unitree_dds_init(C.int(domainID), cIface)
		if rc != 0 {
			return fmt.Errorf("DDS init failed (rc=%d)", rc)
		}
	}
	ddsRefs++
	return nil
}

// ShutdownDDS releases one reference on the global participant.
// When the last reference is released, the participant is deleted and
// the robot is notified immediately (no lease-timeout wait).
func ShutdownDDS() {
	ddsMu.Lock()
	defer ddsMu.Unlock()

	if ddsRefs == 0 {
		return
	}
	ddsRefs--
	if ddsRefs == 0 {
		C.unitree_dds_shutdown()
	}
}

// RPCClient provides request/response communication over a DDS service topic.
type RPCClient struct {
	mu     sync.Mutex
	writer C.dds_entity_t
	reader C.dds_entity_t
	nextID atomic.Int64
}

// NewRPCClient creates an RPC client for the given service (e.g. "sport", "videohub").
func NewRPCClient(serviceName string) (*RPCClient, error) {
	cName := C.CString(serviceName)
	defer C.free(unsafe.Pointer(cName))

	var writer, reader C.dds_entity_t
	rc := C.unitree_dds_create_rpc(cName, &writer, &reader)
	if rc != 0 {
		return nil, fmt.Errorf("create RPC for %q failed (rc=%d)", serviceName, rc)
	}
	return &RPCClient{writer: writer, reader: reader}, nil
}

// Close releases the writer/reader entities. Safe to call multiple times.
func (c *RPCClient) Close() {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.writer == 0 && c.reader == 0 {
		return
	}
	C.unitree_dds_close_rpc(c.writer, c.reader)
	c.writer = 0
	c.reader = 0
}

// Call sends an RPC request and waits for the response.
// Returns the response JSON data and binary payload.
func (c *RPCClient) Call(apiID int64, paramsJSON string, timeoutMs int) (string, []byte, error) {
	reqID := c.nextID.Add(1)

	cParams := C.CString(paramsJSON)
	defer C.free(unsafe.Pointer(cParams))

	c.mu.Lock()
	defer c.mu.Unlock()

	rc := C.unitree_dds_write_request(c.writer, C.int64_t(reqID), C.int64_t(apiID), cParams)
	if rc != 0 {
		return "", nil, fmt.Errorf("write request failed (api=%d rc=%d)", apiID, rc)
	}

	var resp C.unitree_response_t
	rc = C.unitree_dds_read_response(c.reader, C.int(timeoutMs), &resp)
	if rc != 0 {
		return "", nil, fmt.Errorf("read response timeout (api=%d)", apiID)
	}
	defer C.unitree_response_free(&resp)

	var data string
	if resp.data != nil {
		data = C.GoString(resp.data)
	}

	var binary []byte
	if resp.binary._length > 0 && resp.binary._buffer != nil {
		binary = C.GoBytes(unsafe.Pointer(resp.binary._buffer), C.int(resp.binary._length))
	}

	if resp.status_code != 0 {
		return data, binary, fmt.Errorf("RPC error (api=%d status=%d)", apiID, resp.status_code)
	}

	return data, binary, nil
}

// LocoClient wraps the G1 sport service for locomotion commands.
// All methods use the G1-specific API IDs and JSON parameter formats
// (these differ from the Go2 quadruped's API).
type LocoClient struct {
	rpc *RPCClient
}

func NewLocoClient() (*LocoClient, error) {
	rpc, err := NewRPCClient("sport")
	if err != nil {
		return nil, err
	}
	return &LocoClient{rpc: rpc}, nil
}

// SetVelocity sends a velocity command. Duration is in seconds; pass a large
// value (e.g. 864000) for "continuous" movement.
//
// vx is forward velocity (m/s, positive = forward), vy is lateral velocity
// (m/s, positive = left), vyaw is yaw rate (rad/s, positive = counterclockwise).
//
// Note: the G1 robot's velocity-command JSON array is ordered
// [lateral, forward, yaw] despite the C++ SDK's parameter naming suggesting
// otherwise. We swap here so the Go-facing Move(vx, vy, vyaw) keeps
// standard ROS REP-103 semantics (x=forward, y=left).
func (l *LocoClient) SetVelocity(vx, vy, vyaw, duration float32) error {
	params, _ := json.Marshal(map[string]interface{}{
		"velocity": []float32{vy, vx, vyaw},
		"duration": duration,
	})
	_, _, err := l.rpc.Call(ApiLocoSetVelocity, string(params), 1000)
	return err
}

// Move issues a one-shot velocity command (1 second duration). Call repeatedly
// at ~10Hz to maintain motion, or use SetVelocity with a longer duration.
func (l *LocoClient) Move(vx, vy, vyaw float32) error {
	return l.SetVelocity(vx, vy, vyaw, 1.0)
}

// StopMove halts locomotion.
func (l *LocoClient) StopMove() error {
	return l.SetVelocity(0, 0, 0, 1.0)
}

// SetFsmID transitions the robot's finite-state machine to the given state.
func (l *LocoClient) SetFsmID(fsmID int) error {
	params, _ := json.Marshal(map[string]int{"data": fsmID})
	_, _, err := l.rpc.Call(ApiLocoSetFsmID, string(params), 10000)
	return err
}

// SetBalanceMode sets the balance mode (0=static, 1=continuous gait).
func (l *LocoClient) SetBalanceMode(mode int) error {
	params, _ := json.Marshal(map[string]int{"data": mode})
	_, _, err := l.rpc.Call(ApiLocoSetBalanceMode, string(params), 10000)
	return err
}

// SetStandHeight adjusts the standing height.
func (l *LocoClient) SetStandHeight(height float32) error {
	params, _ := json.Marshal(map[string]float32{"data": height})
	_, _, err := l.rpc.Call(ApiLocoSetStandHeight, string(params), 10000)
	return err
}

// SetArmTask triggers a built-in arm action by task ID. The G1 LocoClient
// exposes a fixed set of pre-recorded arm motions (wave, hands-up, hug, etc.).
// See the ArmTask* constants below for known IDs.
func (l *LocoClient) SetArmTask(taskID int) error {
	params, _ := json.Marshal(map[string]int{"data": taskID})
	_, _, err := l.rpc.Call(ApiLocoSetArmTask, string(params), 10000)
	return err
}

// G1 built-in arm action task IDs.
//
// These match the Unitree SDK2 G1 LocoClient pre-recorded arm gestures. The
// numeric IDs come from the SDK's g1_loco_api.hpp / g1_loco_client.hpp.
// "Release" (99) returns the arms to a neutral pose so locomotion can resume.
const (
	ArmTaskReleaseArm  = 99
	ArmTaskShakeHand   = 27
	ArmTaskHighFive    = 18
	ArmTaskHug         = 19
	ArmTaskHeart       = 20
	ArmTaskRefuse      = 21
	ArmTaskRightKiss   = 22
	ArmTaskLeftKiss    = 23
	ArmTaskTwoHandKiss = 24
	ArmTaskHandsUp     = 15
	ArmTaskClap        = 17
	ArmTaskFaceWave    = 12
	ArmTaskHighWave    = 13
	ArmTaskWaveHand    = 0
	ArmTaskTurnWave    = 1
)

// High-level convenience wrappers matching the C++ SDK's LocoClient API.
func (l *LocoClient) ZeroTorque() (int, error)    { return 0, l.SetFsmID(FsmZeroTorque) }
func (l *LocoClient) Damp() (int, error)          { return 0, l.SetFsmID(FsmDamp) }
func (l *LocoClient) Squat() (int, error)         { return 0, l.SetFsmID(FsmSquat) }
func (l *LocoClient) Sit() (int, error)           { return 0, l.SetFsmID(FsmSit) }
func (l *LocoClient) StandUp() (int, error)       { return 0, l.SetFsmID(FsmStandUp) }
func (l *LocoClient) Squat2StandUp() (int, error) { return 0, l.SetFsmID(FsmSquat2StandUp) }
func (l *LocoClient) Lie2StandUp() (int, error)   { return 0, l.SetFsmID(FsmLie2StandUp) }
func (l *LocoClient) Start() (int, error)         { return 0, l.SetFsmID(FsmStart) }
func (l *LocoClient) BalanceStand() (int, error)  { return 0, l.SetBalanceMode(0) }
func (l *LocoClient) HighStand() (int, error)     { return 0, l.SetStandHeight(float32(^uint32(0))) }
func (l *LocoClient) LowStand() (int, error)      { return 0, l.SetStandHeight(0) }

// Arm gesture wrappers.
func (l *LocoClient) WaveHand() (int, error)    { return 0, l.SetArmTask(ArmTaskWaveHand) }
func (l *LocoClient) TurnWave() (int, error)    { return 0, l.SetArmTask(ArmTaskTurnWave) }
func (l *LocoClient) ReleaseArm() (int, error)  { return 0, l.SetArmTask(ArmTaskReleaseArm) }
func (l *LocoClient) ShakeHand() (int, error)   { return 0, l.SetArmTask(ArmTaskShakeHand) }
func (l *LocoClient) HighFive() (int, error)    { return 0, l.SetArmTask(ArmTaskHighFive) }
func (l *LocoClient) Hug() (int, error)         { return 0, l.SetArmTask(ArmTaskHug) }
func (l *LocoClient) Heart() (int, error)       { return 0, l.SetArmTask(ArmTaskHeart) }
func (l *LocoClient) Refuse() (int, error)      { return 0, l.SetArmTask(ArmTaskRefuse) }
func (l *LocoClient) RightKiss() (int, error)   { return 0, l.SetArmTask(ArmTaskRightKiss) }
func (l *LocoClient) LeftKiss() (int, error)    { return 0, l.SetArmTask(ArmTaskLeftKiss) }
func (l *LocoClient) TwoHandKiss() (int, error) { return 0, l.SetArmTask(ArmTaskTwoHandKiss) }
func (l *LocoClient) HandsUp() (int, error)     { return 0, l.SetArmTask(ArmTaskHandsUp) }
func (l *LocoClient) Clap() (int, error)        { return 0, l.SetArmTask(ArmTaskClap) }
func (l *LocoClient) FaceWave() (int, error)    { return 0, l.SetArmTask(ArmTaskFaceWave) }
func (l *LocoClient) HighWave() (int, error)    { return 0, l.SetArmTask(ArmTaskHighWave) }

func (l *LocoClient) Close() {
	if l.rpc != nil {
		l.rpc.Close()
		l.rpc = nil
	}
}

// VideoClient wraps the videohub service for camera capture.
type VideoClient struct {
	rpc *RPCClient
}

func NewVideoClient() (*VideoClient, error) {
	rpc, err := NewRPCClient("videohub")
	if err != nil {
		return nil, err
	}
	return &VideoClient{rpc: rpc}, nil
}

// GetImage captures a JPEG frame from the camera.
func (v *VideoClient) GetImage() ([]byte, error) {
	_, binary, err := v.rpc.Call(ApiGetImageSample, "{}", 5000)
	if err != nil {
		return nil, err
	}
	if len(binary) == 0 {
		return nil, fmt.Errorf("empty image data")
	}
	return binary, nil
}

func (v *VideoClient) Close() {
	if v.rpc != nil {
		v.rpc.Close()
		v.rpc = nil
	}
}

// PointCloud2 is a Go view of a ROS2 sensor_msgs/PointCloud2 message.
type PointCloud2 struct {
	StampSec     int32
	StampNanosec uint32
	FrameID      string
	Height       uint32
	Width        uint32
	Fields       []PointField
	IsBigendian  bool
	PointStep    uint32
	RowStep      uint32
	Data         []byte
	IsDense      bool
}

// PointField describes one field in a PointCloud2 point record.
type PointField struct {
	Name     string
	Offset   uint32
	Datatype uint8
	Count    uint32
}

// PointField datatype enum values (from sensor_msgs/PointField).
const (
	PointFieldInt8    uint8 = 1
	PointFieldUint8   uint8 = 2
	PointFieldInt16   uint8 = 3
	PointFieldUint16  uint8 = 4
	PointFieldInt32   uint8 = 5
	PointFieldUint32  uint8 = 6
	PointFieldFloat32 uint8 = 7
	PointFieldFloat64 uint8 = 8
)

// LidarClient subscribes to a streaming PointCloud2 DDS topic.
type LidarClient struct {
	mu     sync.Mutex
	reader C.dds_entity_t
}

// NewLidarClient creates a subscriber on the given DDS topic
// (e.g. "rt/utlidar/cloud").
func NewLidarClient(topic string) (*LidarClient, error) {
	cTopic := C.CString(topic)
	defer C.free(unsafe.Pointer(cTopic))

	var reader C.dds_entity_t
	rc := C.unitree_dds_subscribe(cTopic, 0 /* PointCloud2 */, &reader)
	if rc != 0 {
		return nil, fmt.Errorf("subscribe to %q failed (rc=%d)", topic, rc)
	}
	return &LidarClient{reader: reader}, nil
}

// Read blocks for up to timeoutMs waiting for the next point cloud.
func (l *LidarClient) Read(timeoutMs int) (*PointCloud2, error) {
	var raw C.unitree_pointcloud2_t
	rc := C.unitree_dds_take_pointcloud2(l.reader, C.int(timeoutMs), &raw)
	if rc != 0 {
		return nil, fmt.Errorf("take pointcloud2 timed out")
	}
	defer C.unitree_pointcloud2_free(&raw)

	pc := &PointCloud2{
		StampSec:     int32(raw.stamp_sec),
		StampNanosec: uint32(raw.stamp_nanosec),
		Height:       uint32(raw.height),
		Width:        uint32(raw.width),
		IsBigendian:  raw.is_bigendian != 0,
		PointStep:    uint32(raw.point_step),
		RowStep:      uint32(raw.row_step),
		IsDense:      raw.is_dense != 0,
	}
	if raw.frame_id != nil {
		pc.FrameID = C.GoString(raw.frame_id)
	}
	if raw.fields._length > 0 && raw.fields._buffer != nil {
		fields := unsafe.Slice((*C.unitree_point_field_t)(unsafe.Pointer(raw.fields._buffer)), int(raw.fields._length))
		pc.Fields = make([]PointField, len(fields))
		for i, f := range fields {
			name := ""
			if f.name != nil {
				name = C.GoString(f.name)
			}
			pc.Fields[i] = PointField{
				Name:     name,
				Offset:   uint32(f.offset),
				Datatype: uint8(f.datatype),
				Count:    uint32(f.count),
			}
		}
	}
	if raw.data._length > 0 && raw.data._buffer != nil {
		pc.Data = C.GoBytes(unsafe.Pointer(raw.data._buffer), C.int(raw.data._length))
	}
	return pc, nil
}

func (l *LidarClient) Close() {
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.reader != 0 {
		C.unitree_dds_close_subscriber(l.reader)
		l.reader = 0
	}
}
