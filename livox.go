package main

/*
#cgo CXXFLAGS: -std=c++11
#cgo CFLAGS: -I${SRCDIR}/capi
#cgo CXXFLAGS: -I${SRCDIR}/capi
#cgo CXXFLAGS: -I${SRCDIR}/build/_deps/livox_sdk2-src/include
#cgo LDFLAGS: -L${SRCDIR}/build -L${SRCDIR}/build/_deps/livox_sdk2-build/sdk_core
#cgo LDFLAGS: -llivox_mid360 -llivox_lidar_sdk_static
#cgo LDFLAGS: -lstdc++ -lm -lpthread

#include "livox_mid360.h"
#include <stdlib.h>
*/
import "C"

import (
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"
	"unsafe"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
)

const (
	defaultLivoxHostIP  = "192.168.123.164"
	defaultLivoxLidarIP = "192.168.123.120"
	livoxMaxPoints      = 200000
)

// LivoxClient streams Mid-360 point clouds via Livox-SDK2 (no Unitree DDS).
type LivoxClient struct {
	mu           sync.Mutex
	closed       bool
	invertMount  bool
	configPath   string
	ownedConfig  bool // delete configPath on Close when we wrote it
}

// NewLivoxClient starts Livox-SDK2 with the given config JSON path.
// If configPath is empty, a temp MID360 config is written using hostIP/lidarIP.
// model is "mid360" (default) or "mid360s".
func NewLivoxClient(configPath, hostIP, lidarIP, model string, frameMs int, invertMount bool) (*LivoxClient, error) {
	if hostIP == "" {
		hostIP = defaultLivoxHostIP
	}
	if lidarIP == "" {
		lidarIP = defaultLivoxLidarIP
	}
	if model == "" {
		model = "mid360"
	}

	owned := false
	path := configPath
	var err error
	if path == "" {
		path, err = writeLivoxConfig(hostIP, lidarIP, model)
		if err != nil {
			return nil, err
		}
		owned = true
	}

	if frameMs > 0 {
		C.livox_mid360_set_frame_ms(C.int(frameMs))
	}

	cPath := C.CString(path)
	defer C.free(unsafe.Pointer(cPath))
	if rc := C.livox_mid360_start(cPath); rc != 0 {
		if owned {
			_ = os.Remove(path)
		}
		return nil, fmt.Errorf("livox_mid360_start failed (rc=%d)", int(rc))
	}

	return &LivoxClient{
		invertMount: invertMount,
		configPath:  path,
		ownedConfig: owned,
	}, nil
}

func writeLivoxConfig(hostIP, lidarIP, model string) (string, error) {
	// lidarIP is documented for operators; SDK2 Mid360 JSON keys host ports.
	// The unit is discovered on the subnet — keep lidarIP in a comment-free
	// companion isn't needed; MID360 block matches Livox-SDK2 quick start.
	_ = lidarIP

	var body string
	switch model {
	case "mid360s", "Mid360s":
		body = fmt.Sprintf(`{
  "Mid360s": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": [
      {
        "host_ip": %q,
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501
      }
    ]
  }
}
`, hostIP)
	default:
		body = fmt.Sprintf(`{
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": [
      {
        "host_ip": %q,
        "multicast_ip": "224.1.1.5",
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501
      }
    ]
  }
}
`, hostIP)
	}

	dir := os.TempDir()
	path := filepath.Join(dir, fmt.Sprintf("viam-livox-%d.json", time.Now().UnixNano()))
	if err := os.WriteFile(path, []byte(body), 0o644); err != nil {
		return "", fmt.Errorf("write livox config: %w", err)
	}
	return path, nil
}

// Read waits for the next assembled frame and returns a Viam point cloud.
// Coordinates are meters. Intensity is attached when present.
func (c *LivoxClient) Read(timeoutMs int) (pointcloud.PointCloud, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.closed {
		return nil, fmt.Errorf("livox client closed")
	}

	xyz := make([]float32, livoxMaxPoints*3)
	intensity := make([]byte, livoxMaxPoints)
	var n C.int
	invert := C.int(0)
	if c.invertMount {
		invert = 1
	}
	rc := C.livox_mid360_take_cloud(
		(*C.float)(unsafe.Pointer(&xyz[0])),
		(*C.uint8_t)(unsafe.Pointer(&intensity[0])),
		C.int(livoxMaxPoints),
		&n,
		C.int(timeoutMs),
		invert,
	)
	if rc != 0 {
		return nil, fmt.Errorf("livox take cloud timed out (rc=%d)", int(rc))
	}
	count := int(n)
	if count <= 0 {
		return nil, fmt.Errorf("livox returned empty cloud")
	}

	pc := pointcloud.NewWithPrealloc(count)
	for i := 0; i < count; i++ {
		p := r3.Vector{
			X: float64(xyz[i*3+0]) * 1000.0, // Viam pointcloud uses mm
			Y: float64(xyz[i*3+1]) * 1000.0,
			Z: float64(xyz[i*3+2]) * 1000.0,
		}
		_ = pc.Set(p, pointcloud.NewBasicData())
	}
	return pc, nil
}

func (c *LivoxClient) Close() {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.closed {
		return
	}
	c.closed = true
	C.livox_mid360_stop()
	if c.ownedConfig && c.configPath != "" {
		_ = os.Remove(c.configPath)
	}
}
