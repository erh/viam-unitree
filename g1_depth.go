package main

import (
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"

	"github.com/blackjack/webcam"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

// g1-depth reads the depth stream of the G1 head camera (Intel RealSense
// D435i) straight from its V4L2 depth node, without librealsense, and serves
// depth images and point clouds. The depth node is separate from the color
// node g1-camera reads, so both can run at once.
var g1DepthModel = resource.NewModel("erh", "viam-unitree", "g1-depth")

// pixelFormatZ16 is V4L2's 'Z16 ' fourcc: 16-bit depth, little-endian.
const pixelFormatZ16 = webcam.PixelFormat('Z' | '1'<<8 | '6'<<16 | ' '<<24)

const (
	defaultDepthMinMM  = 200
	defaultDepthMaxMM  = 5000
	defaultDepthStride = 4
	// Frames older than this are not served.
	g1DepthStaleAfter = time.Second
)

type g1DepthConfig struct {
	// VideoPath picks the depth node; default finds the RealSense node that
	// offers Z16.
	VideoPath string `json:"video_path,omitempty"`
	Width     int    `json:"width,omitempty"`  // default 640
	Height    int    `json:"height,omitempty"` // default 480
	// Intrinsics in pixels; default is a nominal D435i model for the chosen size.
	Fx  float64 `json:"fx,omitempty"`
	Fy  float64 `json:"fy,omitempty"`
	Ppx float64 `json:"ppx,omitempty"`
	Ppy float64 `json:"ppy,omitempty"`
	// DepthScaleMM is millimetres per depth unit (D4xx default 1).
	DepthScaleMM float64 `json:"depth_scale_mm,omitempty"`
	MinDepthMM   float64 `json:"min_depth_mm,omitempty"` // default 200
	MaxDepthMM   float64 `json:"max_depth_mm,omitempty"` // default 5000
	// Stride keeps every Nth pixel in each direction for point clouds (default 4).
	Stride int `json:"stride,omitempty"`
}

func (c *g1DepthConfig) Validate(path string) ([]string, []string, error) {
	if c.Width < 0 || c.Height < 0 || (c.Width == 0) != (c.Height == 0) {
		return nil, nil, fmt.Errorf("%s: set both width and height, or neither", path)
	}
	if c.Fx < 0 || c.Fy < 0 || c.DepthScaleMM < 0 || c.MinDepthMM < 0 || c.MaxDepthMM < 0 || c.Stride < 0 {
		return nil, nil, fmt.Errorf("%s: numeric attributes must not be negative", path)
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(camera.API, g1DepthModel,
		resource.Registration[camera.Camera, *g1DepthConfig]{
			Constructor: newG1Depth,
		})
}

type g1Depth struct {
	resource.Named
	resource.AlwaysRebuild

	logger     logging.Logger
	cam        *webcam.Webcam
	path       string
	width      int
	height     int
	intrinsics depthIntrinsics
	scaleMM    float64
	minMM      float64
	maxMM      float64
	stride     int

	mu       sync.Mutex
	latest   []byte
	latestAt time.Time
	readErr  error

	cancel context.CancelFunc
	wg     sync.WaitGroup
}

func newG1Depth(ctx context.Context, _ resource.Dependencies, conf resource.Config, logger logging.Logger) (camera.Camera, error) {
	cfg, err := resource.NativeConfig[*g1DepthConfig](conf)
	if err != nil {
		return nil, err
	}
	width, height := cfg.Width, cfg.Height
	if width == 0 {
		width, height = defaultDepthWidth, defaultDepthHeight
	}
	in := defaultDepthIntrinsics(width, height)
	if cfg.Fx > 0 {
		in.Fx = cfg.Fx
	}
	if cfg.Fy > 0 {
		in.Fy = cfg.Fy
	}
	if cfg.Ppx > 0 {
		in.Ppx = cfg.Ppx
	}
	if cfg.Ppy > 0 {
		in.Ppy = cfg.Ppy
	}
	d := &g1Depth{
		Named:      conf.ResourceName().AsNamed(),
		logger:     logger,
		width:      width,
		height:     height,
		intrinsics: in,
		scaleMM:    orDefault(cfg.DepthScaleMM, 1),
		minMM:      orDefault(cfg.MinDepthMM, defaultDepthMinMM),
		maxMM:      orDefault(cfg.MaxDepthMM, defaultDepthMaxMM),
		stride:     defaultDepthStride,
	}
	if cfg.Stride > 0 {
		d.stride = cfg.Stride
	}

	candidates := []string{cfg.VideoPath}
	if cfg.VideoPath == "" {
		candidates = realsenseVideoNodes()
	}
	var errs error
	for _, path := range candidates {
		if err := d.open(path); err != nil {
			errs = errors.Join(errs, fmt.Errorf("%s: %w", path, err))
			continue
		}
		break
	}
	if d.cam == nil {
		return nil, fmt.Errorf("no RealSense depth (Z16) V4L2 node found; set video_path: %w", errs)
	}
	logger.Infof("g1-depth: reading %s (Z16 %dx%d, fx=%.1f fy=%.1f ppx=%.1f ppy=%.1f)",
		d.path, d.width, d.height, in.Fx, in.Fy, in.Ppx, in.Ppy)

	readCtx, cancel := context.WithCancel(context.Background())
	d.cancel = cancel
	d.wg.Add(1)
	go d.readLoop(readCtx)
	return d, nil
}

func orDefault(v, def float64) float64 {
	if v > 0 {
		return v
	}
	return def
}

// realsenseVideoNodes lists /dev/videoN nodes whose V4L2 name says RealSense.
// The depth node has no /dev/v4l/by-id link on the G1, so look it up by name
// and let open() keep the one that offers Z16.
func realsenseVideoNodes() []string {
	names, _ := filepath.Glob("/sys/class/video4linux/video*/name")
	var out []string
	for _, n := range names {
		b, err := os.ReadFile(n)
		if err != nil || !strings.Contains(string(b), "RealSense") {
			continue
		}
		out = append(out, "/dev/"+filepath.Base(filepath.Dir(n)))
	}
	sort.Strings(out)
	return out
}

func (d *g1Depth) open(path string) error {
	cam, err := webcam.Open(path)
	if err != nil {
		return err
	}
	if _, ok := cam.GetSupportedFormats()[pixelFormatZ16]; !ok {
		cam.Close()
		return errors.New("no Z16 format")
	}
	_, w, h, err := cam.SetImageFormat(pixelFormatZ16, uint32(d.width), uint32(d.height))
	if err != nil {
		cam.Close()
		return fmt.Errorf("set Z16 %dx%d: %w", d.width, d.height, err)
	}
	if int(w) != d.width || int(h) != d.height {
		cam.Close()
		return fmt.Errorf("driver gave %dx%d, want %dx%d", w, h, d.width, d.height)
	}
	if err := cam.StartStreaming(); err != nil {
		cam.Close()
		return fmt.Errorf("start streaming: %w", err)
	}
	d.cam, d.path = cam, path
	return nil
}

func (d *g1Depth) readLoop(ctx context.Context) {
	defer d.wg.Done()
	for ctx.Err() == nil {
		err := d.cam.WaitForFrame(500)
		var timeout *webcam.Timeout
		if errors.As(err, &timeout) {
			continue
		}
		if err == nil {
			var frame []byte
			var index uint32
			frame, index, err = d.cam.GetFrame()
			if err == nil {
				copied := append([]byte(nil), frame...)
				_ = d.cam.ReleaseFrame(index)
				d.mu.Lock()
				d.latest, d.latestAt, d.readErr = copied, time.Now(), nil
				d.mu.Unlock()
				continue
			}
		}
		d.mu.Lock()
		d.readErr = err
		d.mu.Unlock()
		// Back off so a failing device doesn't spin.
		select {
		case <-ctx.Done():
		case <-time.After(100 * time.Millisecond):
		}
	}
}

func (d *g1Depth) frame() ([]byte, time.Time, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.latest == nil {
		if d.readErr != nil {
			return nil, time.Time{}, fmt.Errorf("no depth frame yet: %w", d.readErr)
		}
		return nil, time.Time{}, errors.New("no depth frame yet")
	}
	if age := time.Since(d.latestAt); age > g1DepthStaleAfter {
		return nil, time.Time{}, fmt.Errorf("depth frame is %.1fs old (last error: %v)", age.Seconds(), d.readErr)
	}
	return d.latest, d.latestAt, nil
}

func (d *g1Depth) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	frame, _, err := d.frame()
	if err != nil {
		return nil, err
	}
	return depthToPointCloud(frame, d.width, d.height, d.intrinsics, d.scaleMM, d.minMM, d.maxMM, d.stride)
}

func (d *g1Depth) Images(
	ctx context.Context, filterSourceNames []string, extra map[string]interface{},
) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	frame, at, err := d.frame()
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	dm := rimage.NewEmptyDepthMap(d.width, d.height)
	for v := 0; v < d.height; v++ {
		for u := 0; u < d.width; u++ {
			i := (v*d.width + u) * 2
			raw := float64(uint16(frame[i]) | uint16(frame[i+1])<<8)
			dm.Set(u, v, rimage.Depth(raw*d.scaleMM))
		}
	}
	ni, err := camera.NamedImageFromImage(dm, d.Name().ShortName(), rdkutils.MimeTypeRawDepth, data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	return []camera.NamedImage{ni}, resource.ResponseMetadata{CapturedAt: at}, nil
}

func (d *g1Depth) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
		ImageType:   camera.DepthStream,
		MimeTypes:   []string{rdkutils.MimeTypeRawDepth},
		IntrinsicParams: &transform.PinholeCameraIntrinsics{
			Width: d.width, Height: d.height,
			Fx: d.intrinsics.Fx, Fy: d.intrinsics.Fy, Ppx: d.intrinsics.Ppx, Ppy: d.intrinsics.Ppy,
		},
	}, nil
}

func (d *g1Depth) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

// DoCommand: {"status": true} reports the device, size, intrinsics and frame age.
func (d *g1Depth) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["status"]; !ok {
		return nil, errors.New(`unknown command; supported: {"status": true}`)
	}
	d.mu.Lock()
	age := -1.0
	if !d.latestAt.IsZero() {
		age = time.Since(d.latestAt).Seconds()
	}
	lastErr := ""
	if d.readErr != nil {
		lastErr = d.readErr.Error()
	}
	d.mu.Unlock()
	return map[string]interface{}{
		"device": d.path, "width": d.width, "height": d.height,
		"fx": d.intrinsics.Fx, "fy": d.intrinsics.Fy, "ppx": d.intrinsics.Ppx, "ppy": d.intrinsics.Ppy,
		"frame_age_s": age, "last_error": lastErr,
	}, nil
}

func (d *g1Depth) Close(ctx context.Context) error {
	if d.cancel != nil {
		d.cancel()
	}
	d.wg.Wait()
	if d.cam == nil {
		return nil
	}
	_ = d.cam.StopStreaming()
	return d.cam.Close()
}
