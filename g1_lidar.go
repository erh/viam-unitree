package main

import (
	"context"
	"encoding/binary"
	"fmt"
	"image"
	"math"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
)

var g1LidarModel = resource.NewModel("erh", "viam-unitree", "g1-lidar")

type G1LidarConfig struct {
	NetworkInterface string `json:"network_interface"`
	// Topic defaults to "rt/utlidar/cloud" (the standard Unitree lidar topic).
	Topic string `json:"topic"`
}

func (c *G1LidarConfig) Validate(path string) ([]string, error) {
	return nil, nil
}

func init() {
	resource.RegisterComponent(camera.API, g1LidarModel, resource.Registration[camera.Camera, *G1LidarConfig]{
		Constructor: newG1Lidar,
	})
}

type g1Lidar struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	lidar  *LidarClient
}

func newG1Lidar(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (camera.Camera, error) {
	cfg, err := resource.NativeConfig[*G1LidarConfig](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}
	topic := "rt/utlidar/cloud"
	if cfg.Topic != "" {
		topic = cfg.Topic
	}

	logger.Infof("Initializing G1Lidar (interface=%s topic=%s)", networkInterface, topic)

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	lidar, err := NewLidarClient(topic)
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("lidar subscribe: %w", err)
	}

	logger.Info("G1Lidar initialized")

	return &g1Lidar{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		lidar:  lidar,
	}, nil
}

// NextPointCloud is the primary API for lidar.
func (l *g1Lidar) NextPointCloud(ctx context.Context) (pointcloud.PointCloud, error) {
	pc2, err := l.lidar.Read(2000)
	if err != nil {
		return nil, err
	}
	return convertPointCloud2(pc2)
}

// convertPointCloud2 turns a ROS2 PointCloud2 into a Viam point cloud.
// Looks up "x", "y", "z" (and optionally "intensity") in the field metadata
// rather than assuming a fixed layout.
func convertPointCloud2(pc *PointCloud2) (pointcloud.PointCloud, error) {
	if len(pc.Data) == 0 || pc.PointStep == 0 {
		return pointcloud.New(), nil
	}

	var xField, yField, zField, intensityField *PointField
	for i := range pc.Fields {
		f := &pc.Fields[i]
		switch f.Name {
		case "x":
			xField = f
		case "y":
			yField = f
		case "z":
			zField = f
		case "intensity":
			intensityField = f
		}
	}
	if xField == nil || yField == nil || zField == nil {
		return nil, fmt.Errorf("PointCloud2 missing required x/y/z fields")
	}

	numPoints := uint32(len(pc.Data)) / pc.PointStep
	out := pointcloud.NewWithPrealloc(int(numPoints))

	var bo binary.ByteOrder = binary.LittleEndian
	if pc.IsBigendian {
		bo = binary.BigEndian
	}

	for i := uint32(0); i < numPoints; i++ {
		base := i * pc.PointStep
		x, ok := readFloat(pc.Data, base+xField.Offset, xField.Datatype, bo)
		if !ok {
			continue
		}
		y, ok := readFloat(pc.Data, base+yField.Offset, yField.Datatype, bo)
		if !ok {
			continue
		}
		z, ok := readFloat(pc.Data, base+zField.Offset, zField.Datatype, bo)
		if !ok {
			continue
		}
		// Skip NaN / invalid points.
		if math.IsNaN(x) || math.IsNaN(y) || math.IsNaN(z) {
			continue
		}

		var data pointcloud.Data
		if intensityField != nil {
			if iv, ok := readFloat(pc.Data, base+intensityField.Offset, intensityField.Datatype, bo); ok {
				data = pointcloud.NewValueData(int(iv))
			}
		}

		// Convert from meters (lidar units) to millimeters (Viam convention).
		_ = out.Set(r3.Vector{X: x * 1000, Y: y * 1000, Z: z * 1000}, data)
	}
	return out, nil
}

func readFloat(buf []byte, off uint32, datatype uint8, bo binary.ByteOrder) (float64, bool) {
	switch datatype {
	case PointFieldFloat32:
		if int(off)+4 > len(buf) {
			return 0, false
		}
		return float64(math.Float32frombits(bo.Uint32(buf[off : off+4]))), true
	case PointFieldFloat64:
		if int(off)+8 > len(buf) {
			return 0, false
		}
		return math.Float64frombits(bo.Uint64(buf[off : off+8])), true
	case PointFieldInt32:
		if int(off)+4 > len(buf) {
			return 0, false
		}
		return float64(int32(bo.Uint32(buf[off : off+4]))), true
	case PointFieldUint32:
		if int(off)+4 > len(buf) {
			return 0, false
		}
		return float64(bo.Uint32(buf[off : off+4])), true
	case PointFieldUint16:
		if int(off)+2 > len(buf) {
			return 0, false
		}
		return float64(bo.Uint16(buf[off : off+2])), true
	case PointFieldUint8:
		if int(off) >= len(buf) {
			return 0, false
		}
		return float64(buf[off]), true
	}
	return 0, false
}

// --- Camera interface methods (mostly no-ops; lidar exposes point clouds, not images) ---

func (l *g1Lidar) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	return nil, camera.ImageMetadata{}, fmt.Errorf("lidar does not produce 2D images; use NextPointCloud")
}

func (l *g1Lidar) Images(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	return nil, resource.ResponseMetadata{CapturedAt: time.Now()}, fmt.Errorf("lidar does not produce 2D images; use NextPointCloud")
}

func (l *g1Lidar) Stream(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
	return gostream.NewEmbeddedVideoStreamFromReader(gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
		return nil, nil, fmt.Errorf("lidar does not produce 2D images")
	})), nil
}

func (l *g1Lidar) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
		ImageType:   camera.UnspecifiedStream,
	}, nil
}

func (l *g1Lidar) Projector(ctx context.Context) (transform.Projector, error) {
	return nil, transform.NewNoIntrinsicsError("")
}

func (l *g1Lidar) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (l *g1Lidar) Close(ctx context.Context) error {
	l.lidar.Close()
	ShutdownDDS()
	return nil
}
