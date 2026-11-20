package main

import (
	"encoding/binary"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

// depthIntrinsics is a pinhole model for the depth image, in pixels.
type depthIntrinsics struct {
	Fx, Fy, Ppx, Ppy float64
}

// D435i depth at 640x480 is close to fx = fy = 383, principal point at the
// image centre. Individual units differ by a few percent, which is fine for
// obstacle avoidance; set the intrinsics in config for anything more precise.
const (
	defaultDepthWidth  = 640
	defaultDepthHeight = 480
	defaultDepthFocal  = 383.0
)

func defaultDepthIntrinsics(width, height int) depthIntrinsics {
	// The nominal focal length is for 640x480; other sizes from the same
	// sensor mode scale with the image height.
	f := defaultDepthFocal * float64(height) / defaultDepthHeight
	return depthIntrinsics{Fx: f, Fy: f, Ppx: float64(width) / 2, Ppy: float64(height) / 2}
}

// depthToPointCloud turns a Z16 frame (little-endian uint16 per pixel) into a
// point cloud in millimetres, in the camera-optical frame (+Z forward, +X
// right, +Y down). Pixels outside [minMM, maxMM] are dropped (0 means no
// depth). stride keeps every stride-th pixel in each direction to bound the
// point count.
func depthToPointCloud(
	frame []byte, width, height int, in depthIntrinsics, depthScaleMM float64, minMM, maxMM float64, stride int,
) (pointcloud.PointCloud, error) {
	if len(frame) < width*height*2 {
		return nil, fmt.Errorf("depth frame is %d bytes, want %d for %dx%d Z16", len(frame), width*height*2, width, height)
	}
	if in.Fx <= 0 || in.Fy <= 0 {
		return nil, fmt.Errorf("depth intrinsics need positive fx and fy, got %v", in)
	}
	if stride < 1 {
		stride = 1
	}
	pc := pointcloud.NewBasicPointCloud((width / stride) * (height / stride))
	for v := 0; v < height; v += stride {
		row := v * width * 2
		for u := 0; u < width; u += stride {
			raw := binary.LittleEndian.Uint16(frame[row+u*2:])
			if raw == 0 {
				continue
			}
			z := float64(raw) * depthScaleMM
			if z < minMM || (maxMM > 0 && z > maxMM) {
				continue
			}
			x := (float64(u) - in.Ppx) * z / in.Fx
			y := (float64(v) - in.Ppy) * z / in.Fy
			if err := pc.Set(r3.Vector{X: x, Y: y, Z: z}, nil); err != nil {
				return nil, err
			}
		}
	}
	return pc, nil
}
