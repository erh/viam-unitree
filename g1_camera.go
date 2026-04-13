package main

import (
	"context"
	"fmt"
	"image"
	"time"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	rdkutils "go.viam.com/rdk/utils"
)

var g1CameraModel = resource.NewModel("erh", "viam-unitree", "g1-camera")

type G1CameraConfig struct {
	NetworkInterface string `json:"network_interface"`
}

func (c *G1CameraConfig) Validate(path string) ([]string, error) {
	return nil, nil
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

	if err := InitChannel(0, networkInterface); err != nil {
		return nil, fmt.Errorf("channel init: %w", err)
	}

	video, err := NewVideoClient()
	if err != nil {
		return nil, fmt.Errorf("video client create: %w", err)
	}
	if err := video.Init(); err != nil {
		video.Close()
		return nil, fmt.Errorf("video client init: %w", err)
	}

	logger.Info("G1Camera initialized successfully")

	return &g1Camera{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		video:  video,
	}, nil
}

func (c *g1Camera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	jpegData, err := c.video.GetImage()
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}
	return jpegData, camera.ImageMetadata{MimeType: rdkutils.MimeTypeJPEG}, nil
}

func (c *g1Camera) Images(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	jpegData, err := c.video.GetImage()
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}

	img, err := rimage.DecodeImage(ctx, jpegData, rdkutils.MimeTypeJPEG)
	if err != nil {
		return nil, resource.ResponseMetadata{}, fmt.Errorf("decode jpeg: %w", err)
	}

	return []camera.NamedImage{
			{Image: img, SourceName: c.Name().ShortName()},
		}, resource.ResponseMetadata{
			CapturedAt: time.Now(),
		}, nil
}

func (c *g1Camera) Stream(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
	return gostream.NewEmbeddedVideoStreamFromReader(gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
		jpegData, err := c.video.GetImage()
		if err != nil {
			return nil, nil, err
		}
		img, err := rimage.DecodeImage(ctx, jpegData, rdkutils.MimeTypeJPEG)
		if err != nil {
			return nil, nil, fmt.Errorf("decode jpeg: %w", err)
		}
		return img, func() {}, nil
	})), nil
}

func (c *g1Camera) NextPointCloud(ctx context.Context) (pointcloud.PointCloud, error) {
	return nil, fmt.Errorf("point cloud not supported")
}

func (c *g1Camera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: false,
		MimeTypes:   []string{rdkutils.MimeTypeJPEG},
		FrameRate:   30.0,
		ImageType:   camera.ColorStream,
	}, nil
}

func (c *g1Camera) Projector(ctx context.Context) (transform.Projector, error) {
	return nil, transform.NewNoIntrinsicsError("")
}

func (c *g1Camera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (c *g1Camera) Close(ctx context.Context) error {
	c.video.Close()
	return nil
}
