package main

import (
	"context"
	"fmt"
	"image"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	"go.uber.org/multierr"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/camera/videosource"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	rdkutils "go.viam.com/rdk/utils"
)

var g1CameraModel = resource.NewModel("erh", "viam-unitree", "g1-camera")

const (
	cameraSourceV4L2     = "v4l2"
	cameraSourceVideohub = "videohub"

	// The RealSense exposes several V4L2 nodes (depth Z16, infrared, color,
	// metadata). Only the color node offers YUYV, so constraining on it
	// selects the RGB stream instead of whichever node enumerates first.
	defaultCameraFormat = "YUY2"
)

type G1CameraConfig struct {
	// Source selects the image backend:
	//   "v4l2" (default): read the G1 head camera (Intel RealSense on the
	//       onboard dev computer) directly as a V4L2 device.
	//   "videohub": Unitree "videohub" DDS RPC service. This is a Go2 API;
	//       stock G1 firmware does not serve it, so requests time out.
	Source string `json:"source,omitempty"`

	// V4L2 options.
	VideoPath string  `json:"video_path,omitempty"`
	Format    string  `json:"format,omitempty"`
	Width     int     `json:"width_px,omitempty"`
	Height    int     `json:"height_px,omitempty"`
	FrameRate float32 `json:"frame_rate,omitempty"`

	// Videohub options.
	NetworkInterface string `json:"network_interface,omitempty"`
}

func (c *G1CameraConfig) Validate(path string) ([]string, error) {
	switch c.Source {
	case "", cameraSourceV4L2, cameraSourceVideohub:
	default:
		return nil, fmt.Errorf("%s: unknown source %q (want %q or %q)", path, c.Source, cameraSourceV4L2, cameraSourceVideohub)
	}
	if c.Width < 0 || c.Height < 0 || c.FrameRate < 0 {
		return nil, fmt.Errorf("%s: width_px, height_px and frame_rate must be non-negative", path)
	}
	return nil, nil
}

func init() {
	resource.RegisterComponent(camera.API, g1CameraModel, resource.Registration[camera.Camera, *G1CameraConfig]{
		Constructor: newG1Camera,
	})
}

func newG1Camera(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (camera.Camera, error) {
	cfg, err := resource.NativeConfig[*G1CameraConfig](conf)
	if err != nil {
		return nil, err
	}
	if cfg.Source == cameraSourceVideohub {
		return newG1VideohubCamera(conf, cfg, logger)
	}
	return newG1V4L2Camera(ctx, deps, conf, cfg, logger)
}

// --- V4L2 backend ---

// newG1V4L2Camera opens the head camera with RDK's webcam driver. With no
// video_path, it tries every RealSense V4L2 node and then any device that
// supports the requested format.
func newG1V4L2Camera(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	cfg *G1CameraConfig,
	logger logging.Logger,
) (camera.Camera, error) {
	format := cfg.Format
	if format == "" {
		format = defaultCameraFormat
	}

	candidates := []string{cfg.VideoPath}
	if cfg.VideoPath == "" {
		candidates = append(findRealSenseVideoDevices(logger), "")
	}

	var errs error
	for _, path := range candidates {
		wcConf := conf
		wcConf.ConvertedAttributes = &videosource.WebcamConfig{
			Path:      path,
			Format:    format,
			Width:     cfg.Width,
			Height:    cfg.Height,
			FrameRate: cfg.FrameRate,
		}
		label := path
		if label == "" {
			label = "<any device>"
		}
		cam, err := videosource.NewWebcam(ctx, deps, wcConf, logger)
		if err == nil {
			logger.Infof("G1Camera using V4L2 device %s (format %s)", label, format)
			return cam, nil
		}
		logger.Debugf("G1Camera: cannot open %s: %v", label, err)
		errs = multierr.Append(errs, fmt.Errorf("%s: %w", label, err))
	}
	return nil, fmt.Errorf("no usable V4L2 camera with format %s found (is the module running on the G1 dev computer with the head camera attached? set video_path to choose a device): %w", format, errs)
}

// findRealSenseVideoDevices returns /dev/videoN paths whose V4L2 name
// mentions RealSense, in numeric order.
func findRealSenseVideoDevices(logger logging.Logger) []string {
	entries, err := filepath.Glob("/sys/class/video4linux/video*")
	if err != nil {
		return nil
	}
	var out []string
	for _, e := range entries {
		name, err := os.ReadFile(filepath.Join(e, "name"))
		if err != nil {
			continue
		}
		n := strings.TrimSpace(string(name))
		logger.Debugf("G1Camera: found %s (%s)", filepath.Base(e), n)
		if strings.Contains(strings.ToLower(n), "realsense") {
			out = append(out, "/dev/"+filepath.Base(e))
		}
	}
	sort.Slice(out, func(i, j int) bool {
		if len(out[i]) != len(out[j]) {
			return len(out[i]) < len(out[j])
		}
		return out[i] < out[j]
	})
	return out
}

// --- videohub backend ---

type g1VideohubCamera struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	video  *VideoClient
}

func newG1VideohubCamera(conf resource.Config, cfg *G1CameraConfig, logger logging.Logger) (camera.Camera, error) {
	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}

	logger.Infof("Initializing G1Camera (videohub) with network interface: %s", networkInterface)

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	video, err := NewVideoClient()
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("video client: %w", err)
	}

	return &g1VideohubCamera{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		video:  video,
	}, nil
}

func (c *g1VideohubCamera) getImage() ([]byte, error) {
	data, err := c.video.GetImage()
	if err != nil {
		return nil, fmt.Errorf("videohub: %w (the G1 does not normally serve videohub; use source \"v4l2\")", err)
	}
	return data, nil
}

func (c *g1VideohubCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	jpegData, err := c.getImage()
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}
	return jpegData, camera.ImageMetadata{MimeType: rdkutils.MimeTypeJPEG}, nil
}

func (c *g1VideohubCamera) Images(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	jpegData, err := c.getImage()
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

func (c *g1VideohubCamera) Stream(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
	return gostream.NewEmbeddedVideoStreamFromReader(gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
		jpegData, err := c.getImage()
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

func (c *g1VideohubCamera) NextPointCloud(ctx context.Context) (pointcloud.PointCloud, error) {
	return nil, fmt.Errorf("point cloud not supported")
}

func (c *g1VideohubCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: false,
		MimeTypes:   []string{rdkutils.MimeTypeJPEG},
		ImageType:   camera.ColorStream,
	}, nil
}

func (c *g1VideohubCamera) Projector(ctx context.Context) (transform.Projector, error) {
	return nil, transform.NewNoIntrinsicsError("")
}

func (c *g1VideohubCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (c *g1VideohubCamera) Close(ctx context.Context) error {
	if c.video == nil {
		return nil
	}
	c.video.Close()
	c.video = nil
	ShutdownDDS()
	return nil
}
