package main

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	"go.uber.org/multierr"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/camera/videosource"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

var g1CameraModel = resource.NewModel("erh", "viam-unitree", "g1-camera")

const (
	cameraSourceV4L2     = "v4l2"
	cameraSourceVideohub = "videohub"

	// The RealSense exposes several V4L2 nodes (depth Z16, infrared, color,
	// metadata). Only the color node offers YUYV, so constraining on it
	// selects the RGB stream regardless of which /dev/videoN it enumerates as
	// (the numbering shuffles across boots). NOTE: this must be the
	// mediadevices frame-format string "YUYV" (V4L2_PIX_FMT_YUYV), NOT "YUY2"
	// — they are distinct constants in mediadevices and the device advertises
	// YUYV.
	defaultCameraFormat = "YUYV"
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

func (c *G1CameraConfig) Validate(path string) ([]string, []string, error) {
	switch c.Source {
	case "", cameraSourceV4L2, cameraSourceVideohub:
	default:
		return nil, nil, fmt.Errorf("%s: unknown source %q (want %q or %q)", path, c.Source, cameraSourceV4L2, cameraSourceVideohub)
	}
	if c.Width < 0 || c.Height < 0 || c.FrameRate < 0 {
		return nil, nil, fmt.Errorf("%s: width_px, height_px and frame_rate must be non-negative", path)
	}
	return nil, nil, nil
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
	// Default / "v4l2". NOTE: on this G1 the RealSense color node is held open
	// by the robot's own vision stack, so the raw device reports "busy" and the
	// only Unitree-side feeds are the videohub RPC (no response topic → times
	// out) and a WebRTC stream — neither is a consumable DDS image topic. v4l2
	// works only once the device is freed (stop whatever holds /dev/video*).
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
	// Formats to try, in order. If the user pinned a format, honor only that.
	// Otherwise try the preferred YUYV color format, then MJPEG, then let the
	// Candidate devices, in preference order: the configured video_path (if
	// any), then every detected RealSense node, then "any device". video_path
	// is a preference, NOT exclusive — if it points at a node that can't give a
	// color image (e.g. the platform /dev/video0 or a depth node), we fall
	// through to the RealSense color node instead of serving black frames.
	var candidates []string
	if cfg.VideoPath != "" {
		candidates = append(candidates, cfg.VideoPath)
	}
	candidates = append(candidates, findRealSenseVideoDevices(logger)...)
	candidates = append(candidates, "") // any device, last resort

	// Formats to try. If the user pinned a format, honor only that. Otherwise
	// try REAL color formats (YUYV then MJPEG) across every candidate first.
	// We deliberately do NOT fall back to letting the driver auto-pick a
	// format: auto-pick happily matches a depth/greyscale/platform node and
	// yields an all-black image, which is worse than a clear error.
	colorFormats := []string{defaultCameraFormat, "MJPEG"}
	if cfg.Format != "" {
		colorFormats = []string{cfg.Format}
	}

	// The RealSense color node won't match a bare format — the webcam driver
	// needs an explicit mode (width/height) to resolve a stream, otherwise
	// every format "fails to find a queryable driver that matches the config
	// constraints". Try a couple of modes the 435i color stream always
	// supports. Config width/height, when set, is used exclusively.
	type mode struct{ w, h int }
	modes := []mode{{cfg.Width, cfg.Height}}
	if cfg.Width == 0 || cfg.Height == 0 {
		modes = []mode{{640, 480}, {1280, 720}}
	}
	frameRate := cfg.FrameRate
	if frameRate == 0 {
		frameRate = 30
	}

	tryOpen := func(path, format string, w, h int) (camera.Camera, error) {
		wcConf := conf
		wcConf.ConvertedAttributes = &videosource.WebcamConfig{
			Path:      path,
			Format:    format,
			Width:     w,
			Height:    h,
			FrameRate: frameRate,
		}
		return videosource.NewWebcam(ctx, deps, wcConf, logger)
	}

	var errs error
	// Format- and mode-outer so a real color mode is tried on the RealSense
	// node before giving up on an earlier (possibly wrong) candidate device.
	for _, format := range colorFormats {
		for _, m := range modes {
			for _, path := range candidates {
				label := path
				if label == "" {
					label = "<any device>"
				}
				cam, err := tryOpen(path, format, m.w, m.h)
				if err != nil {
					logger.Debugf("G1Camera: cannot open %s (%s %dx%d): %v", label, format, m.w, m.h, err)
					errs = multierr.Append(errs, fmt.Errorf("%s/%s/%dx%d open: %w", label, format, m.w, m.h, err))
					continue
				}
				// Opening isn't enough: on a degraded USB link the device opens
				// but never delivers frames (read timeout) — e.g. YUYV 640x480@30
				// is ~18MB/s and won't sustain over USB2, while MJPEG will.
				// Verify we can actually read a frame; if not, release it and try
				// the next format/mode so we land on one that truly streams.
				vctx, cancel := context.WithTimeout(ctx, 4*time.Second)
				_, _, verr := cam.Images(vctx, nil, nil)
				cancel()
				if verr == nil {
					logger.Infof("G1Camera using V4L2 device %s (format %s %dx%d)", label, format, m.w, m.h)
					return cam, nil
				}
				logger.Debugf("G1Camera: %s opened (%s %dx%d) but no frame: %v", label, format, m.w, m.h, verr)
				errs = multierr.Append(errs, fmt.Errorf("%s/%s/%dx%d stream: %w", label, format, m.w, m.h, verr))
				_ = cam.Close(ctx)
			}
		}
	}
	return nil, fmt.Errorf("no usable color V4L2 camera found (is the module running on the G1 dev computer with the head camera attached? set video_path/format to choose a device): %w", errs)
}

// findRealSenseVideoDevices returns candidate RealSense capture-node paths.
//
// It prefers the STABLE /dev/v4l/by-id symlinks over raw /dev/videoN: the
// RealSense re-enumerates to different /dev/videoN numbers across USB resets
// (which happen on power glitches, and when the robot falls), and the webcam
// driver pins to the exact path it opened. A raw /dev/video4 becomes "no such
// device" after a reset; the by-id symlink keeps resolving to the color node,
// so the driver's auto-reconnect works. Only the color node advertises YUYV,
// so the caller's YUYV-first probe still selects the right node among these.
func findRealSenseVideoDevices(logger logging.Logger) []string {
	if byID, err := filepath.Glob("/dev/v4l/by-id/*RealSense*"); err == nil && len(byID) > 0 {
		sort.Strings(byID) // video-index0 (color) first
		for _, e := range byID {
			logger.Debugf("G1Camera: RealSense by-id node %s", e)
		}
		return byID
	}
	// Fallback: /sys name-based /dev/videoN discovery (unstable numbering).
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

func (c *g1VideohubCamera) Images(
	ctx context.Context,
	filterSourceNames []string,
	extra map[string]interface{},
) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	jpegData, err := c.getImage()
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}

	ni, err := camera.NamedImageFromBytes(jpegData, c.Name().ShortName(), rdkutils.MimeTypeJPEG, data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, fmt.Errorf("named image: %w", err)
	}
	return []camera.NamedImage{ni}, resource.ResponseMetadata{CapturedAt: time.Now()}, nil
}

func (c *g1VideohubCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	return nil, fmt.Errorf("point cloud not supported")
}

func (c *g1VideohubCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: false,
		MimeTypes:   []string{rdkutils.MimeTypeJPEG},
		ImageType:   camera.ColorStream,
	}, nil
}

func (c *g1VideohubCamera) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
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
