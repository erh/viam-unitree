package main

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

var g1CameraModel = resource.NewModel("erh", "viam-unitree", "g1-camera")

type G1CameraConfig struct {
	NetworkInterface string `json:"network_interface"`
}

func (c *G1CameraConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(camera.API, g1CameraModel, resource.Registration[camera.Camera, *G1CameraConfig]{
		Constructor: newG1Camera,
	})
}

type g1Camera struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	video  *VideoClient
}

func newG1Camera(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (camera.Camera, error) {
	cfg, err := resource.NativeConfig[*G1CameraConfig](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}

	logger.Infof("Initializing G1Camera with network interface: %s", networkInterface)

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	video, err := NewVideoClient()
	if err != nil {
		return nil, fmt.Errorf("video client: %w", err)
	}

	logger.Info("G1Camera initialized successfully")

	return &g1Camera{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		video:  video,
	}, nil
}

func (c *g1Camera) Images(ctx context.Context, _ []string, _ map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	jpegData, err := c.video.GetImage()
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	img, err := camera.NamedImageFromBytes(jpegData, c.Name().ShortName(), rdkutils.MimeTypeJPEG, data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	return []camera.NamedImage{img}, resource.ResponseMetadata{CapturedAt: time.Now()}, nil
}

func (c *g1Camera) NextPointCloud(ctx context.Context, _ map[string]interface{}) (pointcloud.PointCloud, error) {
	return nil, fmt.Errorf("point cloud not supported")
}

func (c *g1Camera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func (c *g1Camera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: false,
		MimeTypes:   []string{rdkutils.MimeTypeJPEG},
		FrameRate:   30.0,
		ImageType:   camera.ColorStream,
	}, nil
}

func (c *g1Camera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (c *g1Camera) Close(ctx context.Context) error {
	c.video.Close()
	ShutdownDDS()
	return nil
}
