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

// Unitree sport API IDs.
const (
	ApiDamp         int64 = 1001
	ApiBalanceStand int64 = 1002
	ApiStopMove     int64 = 1003
	ApiStandUp      int64 = 1004
	ApiStandDown    int64 = 1005
	ApiEuler        int64 = 1007
	ApiMove         int64 = 1008
	ApiSit          int64 = 1009
	ApiRiseSit      int64 = 1010
	ApiHello        int64 = 1016
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

// LocoClient wraps the sport service for locomotion commands.
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

func (l *LocoClient) Move(vx, vy, vyaw float32) error {
	params, _ := json.Marshal(map[string]float32{"vx": vx, "vy": vy, "vyaw": vyaw})
	_, _, err := l.rpc.Call(ApiMove, string(params), 1000)
	return err
}

func (l *LocoClient) StopMove() error {
	_, _, err := l.rpc.Call(ApiStopMove, "{}", 1000)
	return err
}

func (l *LocoClient) StandUp() (int, error)      { return l.simpleCall(ApiStandUp) }
func (l *LocoClient) Sit() (int, error)           { return l.simpleCall(ApiSit) }
func (l *LocoClient) BalanceStand() (int, error)   { return l.simpleCall(ApiBalanceStand) }
func (l *LocoClient) Damp() (int, error)           { return l.simpleCall(ApiDamp) }
func (l *LocoClient) StandDown() (int, error)      { return l.simpleCall(ApiStandDown) }
func (l *LocoClient) RiseSit() (int, error)        { return l.simpleCall(ApiRiseSit) }
func (l *LocoClient) Hello() (int, error)          { return l.simpleCall(ApiHello) }

func (l *LocoClient) simpleCall(apiID int64) (int, error) {
	data, _, err := l.rpc.Call(apiID, "{}", 10000)
	if err != nil {
		return -1, err
	}
	_ = data
	return 0, nil
}

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
