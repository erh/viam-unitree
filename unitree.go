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
	"strings"
	"sync"
	"sync/atomic"
	"time"
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
// Which IDs are honored depends on firmware version. The Unitree Python
// SDK documents the newer transitional IDs (702=Lie2StandUp,
// 706=Squat2StandUp, 200=Start), while the older C++ SDK used
// FsmStandUp=4 and FsmStart=500. This robot's firmware only accepts
// the legacy ID 4 for stand-up — 706/200/500/702 are silently rejected.
// Use try_fsm DoCommand to discover which IDs a given firmware honors.
const (
	FsmZeroTorque    = 0
	FsmDamp          = 1
	FsmSquat         = 2
	FsmSit           = 3
	FsmStandUp       = 4 // the stand-up transition this firmware honors
	FsmStart         = 200
	FsmLie2StandUp   = 702
	FsmSquat2StandUp = 706
	FsmRun           = 802 // walk/run mode — Move commands work here
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

	// strictMatch controls request/response correlation. When true
	// (e.g. sport service), Call drains stale responses at the start
	// and filters incoming responses by identity_api_id / identity_id
	// so stale replies from prior requests can't be mis-attributed.
	// When false (e.g. videohub), Call takes the next response
	// regardless — videohub doesn't reliably echo request identifiers
	// and can have late-arriving frames that must not be drained.
	strictMatch bool
}

// NewRPCClient creates an RPC client for the given service. Pass
// strictMatch=true for services where request/response identity is
// tracked (sport), false for services that don't echo identifiers
// (videohub).
func NewRPCClient(serviceName string, strictMatch bool) (*RPCClient, error) {
	cName := C.CString(serviceName)
	defer C.free(unsafe.Pointer(cName))

	var writer, reader C.dds_entity_t
	rc := C.unitree_dds_create_rpc(cName, &writer, &reader)
	if rc != 0 {
		return nil, fmt.Errorf("create RPC for %q failed (rc=%d)", serviceName, rc)
	}
	return &RPCClient{writer: writer, reader: reader, strictMatch: strictMatch}, nil
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

// Call sends an RPC request and waits for the matching response.
// Returns the response JSON data and binary payload.
//
// The DDS reader can hold stale responses from previous calls (notably
// across a robot reboot, since the Go-side DDS participant stays alive).
// We loop on read and discard any response whose identity_id doesn't
// match the request we just sent, until the matching one arrives or the
// total timeout elapses.
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

	deadline := time.Now().Add(time.Duration(timeoutMs) * time.Millisecond)
	for {
		remaining := time.Until(deadline)
		if remaining <= 0 {
			return "", nil, fmt.Errorf("read response timeout (api=%d req_id=%d)", apiID, reqID)
		}

		var resp C.unitree_response_t
		rc = C.unitree_dds_read_response(c.reader, C.int(remaining.Milliseconds()), &resp)
		if rc != 0 {
			return "", nil, fmt.Errorf("read response timeout (api=%d req_id=%d)", apiID, reqID)
		}

		if c.strictMatch {
			respAPIID := int64(resp.identity_api_id)
			respReqID := int64(resp.identity_id)
			if (respAPIID != 0 && respAPIID != apiID) || (respReqID != 0 && respReqID != reqID) {
				C.unitree_response_free(&resp)
				continue
			}
		}

		var data string
		if resp.data != nil {
			data = C.GoString(resp.data)
		}
		var binary []byte
		if resp.binary._length > 0 && resp.binary._buffer != nil {
			binary = C.GoBytes(unsafe.Pointer(resp.binary._buffer), C.int(resp.binary._length))
		}
		status := resp.status_code
		C.unitree_response_free(&resp)

		if status != 0 {
			return data, binary, fmt.Errorf("RPC error (api=%d status=%d)", apiID, status)
		}
		return data, binary, nil
	}
}

// LocoClient wraps the G1 sport service for locomotion commands.
// All methods use the G1-specific API IDs and JSON parameter formats
// (these differ from the Go2 quadruped's API).
type LocoClient struct {
	rpc *RPCClient
}

func NewLocoClient() (*LocoClient, error) {
	rpc, err := NewRPCClient("sport", true)
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

// dataIntResponse matches the JSON shape used by G1 getters
// (e.g. GetFsmId / GetFsmMode / GetBalanceMode): {"data": <int>}.
type dataIntResponse struct {
	Data int `json:"data"`
}

// GetFsmID reads the robot's current FSM state ID.
func (l *LocoClient) GetFsmID() (int, error) {
	data, _, err := l.rpc.Call(ApiLocoGetFsmID, "", 10000)
	if err != nil {
		return 0, err
	}
	var r dataIntResponse
	if err := json.Unmarshal([]byte(data), &r); err != nil {
		return 0, fmt.Errorf("parse FSM ID response %q: %w", data, err)
	}
	return r.Data, nil
}

// GetFsmMode reads the robot's current FSM mode.
func (l *LocoClient) GetFsmMode() (int, error) {
	data, _, err := l.rpc.Call(ApiLocoGetFsmMode, "", 10000)
	if err != nil {
		return 0, err
	}
	var r dataIntResponse
	if err := json.Unmarshal([]byte(data), &r); err != nil {
		return 0, fmt.Errorf("parse FSM mode response %q: %w", data, err)
	}
	return r.Data, nil
}

// GetBalanceMode reads the robot's current balance mode (0=static,
// 1=continuous gait).
func (l *LocoClient) GetBalanceMode() (int, error) {
	data, _, err := l.rpc.Call(ApiLocoGetBalanceMode, "", 10000)
	if err != nil {
		return 0, err
	}
	var r dataIntResponse
	if err := json.Unmarshal([]byte(data), &r); err != nil {
		return 0, fmt.Errorf("parse balance mode response %q: %w", data, err)
	}
	return r.Data, nil
}

// dataFloatResponse matches the JSON shape for float getters:
// {"data": <float>}.
type dataFloatResponse struct {
	Data float64 `json:"data"`
}

// GetSwingHeight reads the robot's current swing height (meters).
func (l *LocoClient) GetSwingHeight() (float64, error) {
	data, _, err := l.rpc.Call(ApiLocoGetSwingHeight, "", 10000)
	if err != nil {
		return 0, err
	}
	var r dataFloatResponse
	if err := json.Unmarshal([]byte(data), &r); err != nil {
		return 0, fmt.Errorf("parse swing height response %q: %w", data, err)
	}
	return r.Data, nil
}

// GetStandHeight reads the robot's current stand height (meters).
func (l *LocoClient) GetStandHeight() (float64, error) {
	data, _, err := l.rpc.Call(ApiLocoGetStandHeight, "", 10000)
	if err != nil {
		return 0, err
	}
	var r dataFloatResponse
	if err := json.Unmarshal([]byte(data), &r); err != nil {
		return 0, fmt.Errorf("parse stand height response %q: %w", data, err)
	}
	return r.Data, nil
}

// SetSpeedMode sets the robot's speed mode. API 7107; semantics and
// valid range are firmware-dependent.
func (l *LocoClient) SetSpeedMode(mode int) error {
	params, _ := json.Marshal(map[string]int{"data": mode})
	_, _, err := l.rpc.Call(ApiLocoSetSpeedMode, string(params), 10000)
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
func (l *LocoClient) Run() (int, error)           { return 0, l.SetFsmID(FsmRun) }
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
	rpc, err := NewRPCClient("videohub", false)
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

// Unitree SLAM (slam_operate) service API IDs. From the Unitree "SLAM and
// Navigation Services Interface" docs (service name "slam_operate", v1.0.0.1).
// On the G1 the lidar driver publishes rt/utlidar/cloud only while the SLAM
// stack is active, so starting mapping is what brings the point cloud online.
const (
	ApiSlamStartMapping int64 = 1801 // params: {"data":{"slam_type":"indoor"}}
	ApiSlamEndMapping   int64 = 1802
	ApiSlamInitPose     int64 = 1804
	ApiSlamCloseSlam    int64 = 1901
)

// SlamClient controls the robot's LiDAR SLAM stack over the "slam_operate" RPC
// service.
type SlamClient struct {
	rpc *RPCClient
}

// NewSlamClient creates a client for the slam_operate service.
func NewSlamClient() (*SlamClient, error) {
	rpc, err := NewRPCClient("slam_operate", true)
	if err != nil {
		return nil, err
	}
	return &SlamClient{rpc: rpc}, nil
}

// Operate issues a slam_operate call. paramsJSON may be "" for calls that take
// no parameters. timeoutMs should be generous — starting SLAM spins up the
// lidar driver and can take tens of seconds to reply. Returns the response JSON.
func (s *SlamClient) Operate(apiID int64, paramsJSON string, timeoutMs int) (string, error) {
	data, _, err := s.rpc.Call(apiID, paramsJSON, timeoutMs)
	return data, err
}

func (s *SlamClient) Close() {
	if s.rpc != nil {
		s.rpc.Close()
	}
}

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

// ListPublications returns every remote publication the DDS participant has
// discovered, as "topic_name|type_name" strings. Useful for diagnosing whether
// an expected topic (e.g. rt/utlidar/cloud) is actually being published and
// reachable on the bound network interface.
func ListPublications() ([]string, error) {
	const bufSize = 64 * 1024
	buf := make([]byte, bufSize)
	n := C.unitree_dds_list_publications((*C.char)(unsafe.Pointer(&buf[0])), C.int(bufSize))
	if n < 0 {
		return nil, fmt.Errorf("list publications failed")
	}
	s := C.GoString((*C.char)(unsafe.Pointer(&buf[0])))
	if s == "" {
		return []string{}, nil
	}
	return strings.Split(strings.TrimRight(s, "\n"), "\n"), nil
}

// ListSubscriptions returns every remote subscription (reader) the DDS
// participant has discovered, as "topic_name|type_name" strings. Useful to
// check whether a node is alive even when it publishes nothing (e.g. the
// utlidar switch consumer).
func ListSubscriptions() ([]string, error) {
	const bufSize = 64 * 1024
	buf := make([]byte, bufSize)
	n := C.unitree_dds_list_subscriptions((*C.char)(unsafe.Pointer(&buf[0])), C.int(bufSize))
	if n < 0 {
		return nil, fmt.Errorf("list subscriptions failed")
	}
	s := C.GoString((*C.char)(unsafe.Pointer(&buf[0])))
	if s == "" {
		return []string{}, nil
	}
	return strings.Split(strings.TrimRight(s, "\n"), "\n"), nil
}

// StringWriter publishes std_msgs/String messages on a DDS topic. It is used
// for simple control topics such as the utlidar switch ("rt/utlidar/switch"),
// where writing "ON"/"OFF" enables/disables the lidar point-cloud stream.
type StringWriter struct {
	mu     sync.Mutex
	writer C.dds_entity_t
}

// NewStringWriter creates a publisher on the given DDS topic.
func NewStringWriter(topic string) (*StringWriter, error) {
	cTopic := C.CString(topic)
	defer C.free(unsafe.Pointer(cTopic))

	var writer C.dds_entity_t
	rc := C.unitree_dds_create_string_writer(cTopic, &writer)
	if rc != 0 {
		return nil, fmt.Errorf("create string writer for %q failed (rc=%d)", topic, rc)
	}
	return &StringWriter{writer: writer}, nil
}

// Publish writes a single std_msgs/String sample.
func (s *StringWriter) Publish(data string) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.writer == 0 {
		return fmt.Errorf("string writer closed")
	}
	cData := C.CString(data)
	defer C.free(unsafe.Pointer(cData))
	if rc := C.unitree_dds_publish_string(s.writer, cData); rc != 0 {
		return fmt.Errorf("publish string failed (rc=%d)", rc)
	}
	return nil
}

func (s *StringWriter) Close() {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.writer != 0 {
		C.unitree_dds_close_writer(s.writer)
		s.writer = 0
	}
}

// OdomState is a snapshot of the G1 odometer (SportModeState) fields we expose.
type OdomState struct {
	Position     [3]float32 // world-frame meters: x forward, y left, z up
	Velocity     [3]float32 // world-frame m/s
	RPY          [3]float32 // roll, pitch, yaw (rad)
	Quaternion   [4]float32 // w, x, y, z
	Gyro         [3]float32 // rad/s
	Accel        [3]float32 // m/s^2
	YawSpeed     float32    // body-frame yaw rate, rad/s
	BodyHeight   float32
	ErrorCode    uint32
	StampSec     int32
	StampNanosec uint32
}

// OdometerClient subscribes to a SportModeState DDS topic and caches the
// latest sample on a background poller — the same pattern as arm_sdk's
// rt/lowstate reader (not the lidar PointCloud2 take-on-demand path).
type OdometerClient struct {
	mu     sync.Mutex
	reader C.dds_entity_t
	closed bool

	latest  OdomState
	hasData atomic.Bool
	stopCh  chan struct{}
	wg      sync.WaitGroup
}

// NewOdometerClient subscribes to topic (e.g. "rt/lf/odommodestate" or
// "rt/odommodestate") and starts a background poller.
func NewOdometerClient(topic string) (*OdometerClient, error) {
	cTopic := C.CString(topic)
	defer C.free(unsafe.Pointer(cTopic))

	var reader C.dds_entity_t
	rc := C.unitree_dds_subscribe(cTopic, 2 /* SportModeState */, &reader)
	if rc != 0 {
		return nil, fmt.Errorf("subscribe to %q failed (rc=%d)", topic, rc)
	}

	c := &OdometerClient{
		reader: reader,
		stopCh: make(chan struct{}),
	}
	c.wg.Add(1)
	go c.poll()
	return c, nil
}

func (c *OdometerClient) poll() {
	defer c.wg.Done()
	ticker := time.NewTicker(5 * time.Millisecond) // enough for 20Hz or 500Hz
	defer ticker.Stop()

	for {
		select {
		case <-c.stopCh:
			return
		case <-ticker.C:
		}

		var raw C.unitree_go_sport_mode_state_t
		// Non-blocking take (timeout 0), same as arm_sdk lowstate polling.
		if rc := C.unitree_dds_take_sport_mode_state(c.reader, 0, &raw); rc != 0 {
			continue
		}

		st := OdomState{
			YawSpeed:     float32(raw.yaw_speed),
			BodyHeight:   float32(raw.body_height),
			ErrorCode:    uint32(raw.error_code),
			StampSec:     int32(raw.stamp_sec),
			StampNanosec: uint32(raw.stamp_nanosec),
		}
		for i := 0; i < 3; i++ {
			st.Position[i] = float32(raw.position[i])
			st.Velocity[i] = float32(raw.velocity[i])
			st.RPY[i] = float32(raw.imu_state.rpy[i])
			st.Gyro[i] = float32(raw.imu_state.gyroscope[i])
			st.Accel[i] = float32(raw.imu_state.accelerometer[i])
		}
		for i := 0; i < 4; i++ {
			st.Quaternion[i] = float32(raw.imu_state.quaternion[i])
		}

		c.mu.Lock()
		c.latest = st
		c.hasData.Store(true)
		c.mu.Unlock()
	}
}

// Latest returns the most recent odometer sample.
func (c *OdometerClient) Latest() (OdomState, error) {
	if !c.hasData.Load() {
		return OdomState{}, fmt.Errorf("no odometer data received yet")
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.latest, nil
}

func (c *OdometerClient) Close() {
	c.mu.Lock()
	if c.closed {
		c.mu.Unlock()
		return
	}
	c.closed = true
	reader := c.reader
	c.mu.Unlock()

	close(c.stopCh)
	c.wg.Wait()
	C.unitree_dds_close_subscriber(reader)

	c.mu.Lock()
	c.reader = 0
	c.mu.Unlock()
}
