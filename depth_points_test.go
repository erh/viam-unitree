package main

import (
	"encoding/binary"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

func z16Frame(width, height int, depth func(u, v int) uint16) []byte {
	frame := make([]byte, width*height*2)
	for v := 0; v < height; v++ {
		for u := 0; u < width; u++ {
			binary.LittleEndian.PutUint16(frame[(v*width+u)*2:], depth(u, v))
		}
	}
	return frame
}

func points(pc pointcloud.PointCloud) []r3.Vector {
	var out []r3.Vector
	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		out = append(out, p)
		return true
	})
	return out
}

func TestDepthToPointCloudProjectsPinhole(t *testing.T) {
	in := depthIntrinsics{Fx: 100, Fy: 100, Ppx: 2, Ppy: 1}
	// One pixel at (u=4, v=3) is 1000 mm away; everything else has no depth.
	frame := z16Frame(5, 4, func(u, v int) uint16 {
		if u == 4 && v == 3 {
			return 1000
		}
		return 0
	})
	pc, err := depthToPointCloud(frame, 5, 4, in, 1.0, 0, 0, 1)
	if err != nil {
		t.Fatal(err)
	}
	got := points(pc)
	if len(got) != 1 {
		t.Fatalf("got %d points, want 1", len(got))
	}
	// x = (4-2)*1000/100 = 20 mm right, y = (3-1)*1000/100 = 20 mm down, z = 1000 mm forward.
	want := r3.Vector{X: 20, Y: 20, Z: 1000}
	if got[0].Sub(want).Norm() > 1e-9 {
		t.Fatalf("got %v, want %v", got[0], want)
	}
}

func TestDepthToPointCloudFiltersRangeAndStrides(t *testing.T) {
	in := defaultDepthIntrinsics(8, 8)
	frame := z16Frame(8, 8, func(u, v int) uint16 {
		switch {
		case u == 0:
			return 100 // too close
		case u == 1:
			return 9000 // too far
		default:
			return 2000
		}
	})
	pc, err := depthToPointCloud(frame, 8, 8, in, 1.0, 200, 5000, 2)
	if err != nil {
		t.Fatal(err)
	}
	// Stride 2 keeps columns 0,2,4,6 and rows 0,2,4,6; column 0 is too close.
	if n := len(points(pc)); n != 12 {
		t.Fatalf("got %d points, want 12", n)
	}
	for _, p := range points(pc) {
		if math.Abs(p.Z-2000) > 1e-9 {
			t.Fatalf("unexpected depth %v", p.Z)
		}
	}
}

func TestDepthToPointCloudRejectsShortFrames(t *testing.T) {
	if _, err := depthToPointCloud(make([]byte, 10), 4, 4, defaultDepthIntrinsics(4, 4), 1, 0, 0, 1); err == nil {
		t.Fatal("expected an error for a short frame")
	}
}

func TestDefaultDepthIntrinsicsAt640x480(t *testing.T) {
	in := defaultDepthIntrinsics(640, 480)
	if in.Fx != 383 || in.Fy != 383 || in.Ppx != 320 || in.Ppy != 240 {
		t.Fatalf("got %+v", in)
	}
}
