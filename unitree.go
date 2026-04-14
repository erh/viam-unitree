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
	ApiLocoGetFsmID        int64 = 7001
	ApiLocoGetFsmMode      int64 = 7002
	ApiLocoGetBalanceMode  int64 = 7003
	ApiLocoGetSwingHeight  int64 = 7004
	ApiLocoGetStandHeight  int64 = 7005
	ApiLocoSetFsmID        int64 = 7101
	ApiLocoSetBalanceMode  int64 = 7102
	ApiLocoSetSwingHeight  int64 = 7103
	ApiLocoSetStandHeight  int64 = 7104
	ApiLocoSetVelocity     int64 = 7105
	ApiLocoSetArmTask      int64 = 7106
	ApiLocoSetSpeedMode    int64 = 7107
)

// G1 FSM (Finite State Machine) IDs used with SetFsmId.
const (
	FsmZeroTorque  = 0
	FsmDamp        = 1
	FsmSquat       = 2
	FsmSit         = 3
	FsmStandUp     = 4
	FsmStart       = 500
)

// Unitree video API IDs.
const (
	ApiGetImageSample int64 = 1001
)

// InitDDS initializes the DDS participant.
func InitDDS(domainID int, networkInterface string) error {
	cIface := C.CString(networkInterface)
	defer C.free(unsafe.Pointer(cIface))
	rc := C.unitree_dds_init(C.int(domainID), cIface)
	if rc != 0 {
		return fmt.Errorf("DDS init failed (rc=%d)", rc)
	}
	return nil
}

// ShutdownDDS shuts down the DDS participant.
func ShutdownDDS() {
	C.unitree_dds_shutdown()
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
func (l *LocoClient) SetVelocity(vx, vy, vyaw, duration float32) error {
	params, _ := json.Marshal(map[string]interface{}{
		"velocity": []float32{vx, vy, vyaw},
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

// SetArmTask triggers an arm task by ID (e.g. wave_hand=0, turn_wave=1).
func (l *LocoClient) SetArmTask(taskID int) error {
	params, _ := json.Marshal(map[string]int{"data": taskID})
	_, _, err := l.rpc.Call(ApiLocoSetArmTask, string(params), 10000)
	return err
}

// High-level convenience wrappers matching the C++ SDK's LocoClient API.
func (l *LocoClient) ZeroTorque() (int, error)   { return 0, l.SetFsmID(FsmZeroTorque) }
func (l *LocoClient) Damp() (int, error)         { return 0, l.SetFsmID(FsmDamp) }
func (l *LocoClient) Squat() (int, error)        { return 0, l.SetFsmID(FsmSquat) }
func (l *LocoClient) Sit() (int, error)          { return 0, l.SetFsmID(FsmSit) }
func (l *LocoClient) StandUp() (int, error)      { return 0, l.SetFsmID(FsmStandUp) }
func (l *LocoClient) Start() (int, error)        { return 0, l.SetFsmID(FsmStart) }
func (l *LocoClient) BalanceStand() (int, error) { return 0, l.SetBalanceMode(0) }
func (l *LocoClient) HighStand() (int, error)    { return 0, l.SetStandHeight(float32(^uint32(0))) }
func (l *LocoClient) LowStand() (int, error)     { return 0, l.SetStandHeight(0) }
func (l *LocoClient) WaveHand() (int, error)     { return 0, l.SetArmTask(0) }

func (l *LocoClient) Close() {}

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

func (v *VideoClient) Close() {}
