package main

/*
#cgo CFLAGS: -I${SRCDIR}/capi
#cgo LDFLAGS: -L${SRCDIR}/build -lunitree_capi
#cgo LDFLAGS: -L${SRCDIR}/build/_deps/unitree_sdk2-src/lib/x86_64 -lunitree_sdk2
#cgo LDFLAGS: -L${SRCDIR}/build/_deps/unitree_sdk2-src/thirdparty/lib/x86_64 -lddsc -lddscxx
#cgo LDFLAGS: -lstdc++ -lm -lpthread
#cgo LDFLAGS: -Wl,-rpath,${SRCDIR}/build/_deps/unitree_sdk2-src/thirdparty/lib/x86_64

#include "unitree_capi.h"
#include <stdlib.h>
*/
import "C"

import (
	"fmt"
	"unsafe"
)

// InitChannel initializes the Unitree DDS channel factory.
func InitChannel(domainID int, networkInterface string) error {
	cIface := C.CString(networkInterface)
	defer C.free(unsafe.Pointer(cIface))
	rc := C.unitree_channel_init(C.int(domainID), cIface)
	if rc != 0 {
		return fmt.Errorf("unitree channel init failed (rc=%d)", rc)
	}
	return nil
}

// LocoClient wraps the Unitree G1 locomotion client.
type LocoClient struct {
	handle C.unitree_loco_client_t
}

// NewLocoClient creates a new locomotion client. Must call Init() before use.
func NewLocoClient() (*LocoClient, error) {
	h := C.unitree_loco_new()
	if h == nil {
		return nil, fmt.Errorf("failed to create loco client")
	}
	return &LocoClient{handle: h}, nil
}

func (l *LocoClient) Init() error {
	if rc := C.unitree_loco_init(l.handle); rc != 0 {
		return fmt.Errorf("loco init failed (rc=%d)", rc)
	}
	return nil
}

func (l *LocoClient) SetTimeout(sec float32) {
	C.unitree_loco_set_timeout(l.handle, C.float(sec))
}

func (l *LocoClient) Move(vx, vy, vyaw float32) int {
	return int(C.unitree_loco_move(l.handle, C.float(vx), C.float(vy), C.float(vyaw)))
}

func (l *LocoClient) StopMove() int {
	return int(C.unitree_loco_stop_move(l.handle))
}

func (l *LocoClient) StandUp() int    { return int(C.unitree_loco_stand_up(l.handle)) }
func (l *LocoClient) Sit() int         { return int(C.unitree_loco_sit(l.handle)) }
func (l *LocoClient) Squat() int       { return int(C.unitree_loco_squat(l.handle)) }
func (l *LocoClient) HighStand() int   { return int(C.unitree_loco_high_stand(l.handle)) }
func (l *LocoClient) LowStand() int    { return int(C.unitree_loco_low_stand(l.handle)) }
func (l *LocoClient) BalanceStand() int { return int(C.unitree_loco_balance_stand(l.handle)) }
func (l *LocoClient) Damp() int        { return int(C.unitree_loco_damp(l.handle)) }
func (l *LocoClient) ZeroTorque() int  { return int(C.unitree_loco_zero_torque(l.handle)) }
func (l *LocoClient) WaveHand() int    { return int(C.unitree_loco_wave_hand(l.handle)) }
func (l *LocoClient) Start() int       { return int(C.unitree_loco_start(l.handle)) }

func (l *LocoClient) Close() {
	if l.handle != nil {
		C.unitree_loco_free(l.handle)
		l.handle = nil
	}
}

// VideoClient wraps the Unitree video capture client.
type VideoClient struct {
	handle C.unitree_video_client_t
}

// NewVideoClient creates a new video client. Must call Init() before use.
func NewVideoClient() (*VideoClient, error) {
	h := C.unitree_video_new()
	if h == nil {
		return nil, fmt.Errorf("failed to create video client")
	}
	return &VideoClient{handle: h}, nil
}

func (v *VideoClient) Init() error {
	if rc := C.unitree_video_init(v.handle); rc != 0 {
		return fmt.Errorf("video init failed (rc=%d)", rc)
	}
	return nil
}

// GetImage captures a JPEG frame from the camera.
func (v *VideoClient) GetImage() ([]byte, error) {
	var data *C.uint8_t
	var size C.int
	rc := C.unitree_video_get_image(v.handle, &data, &size)
	if rc != 0 {
		return nil, fmt.Errorf("GetImageSample failed (rc=%d)", rc)
	}
	defer C.unitree_image_free(data)
	return C.GoBytes(unsafe.Pointer(data), size), nil
}

func (v *VideoClient) Close() {
	if v.handle != nil {
		C.unitree_video_free(v.handle)
		v.handle = nil
	}
}
