package main

import (
	"bytes"
	"context"
	"encoding/binary"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/png"
	"math"
	"strings"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	rdkutils "go.viam.com/rdk/utils"
)

var g1LidarModel = resource.NewModel("erh", "viam-unitree", "g1-lidar")

type G1LidarConfig struct {
	NetworkInterface string `json:"network_interface"`
	// Topic defaults to "rt/utlidar/cloud" (the standard Unitree lidar topic).
	Topic string `json:"topic"`

	// SwitchTopic is the std_msgs/String control topic used to turn the lidar
	// on/off. Defaults to "rt/utlidar/switch". Set to "-" to disable switch
	// control entirely (e.g. if the lidar is managed elsewhere).
	SwitchTopic string `json:"switch_topic"`

	// DisableSwitchOnStartup, when true, skips publishing "ON" to the switch
	// topic at startup. By default the component turns the lidar on when it
	// initializes (the utlidar only publishes point clouds while enabled).
	DisableSwitchOnStartup bool `json:"disable_switch_on_startup"`

	// StartSlamOnStartup, when true, calls the SLAM start API on init. On the G1
	// the lidar driver publishes rt/utlidar/cloud only while the SLAM stack is
	// active, so this is what actually brings the point cloud online.
	StartSlamOnStartup bool `json:"start_slam_on_startup"`

	// SlamStartAPIID / SlamStartParams / SlamStopAPIID control the SLAM calls.
	// Defaults follow Unitree's slam_operate docs: start=1801 (start mapping)
	// with params {"data":{"slam_type":"indoor"}}, stop=1901 (close slam).
	// Override if your firmware uses different IDs (check ~/unitree_sdk2 on the
	// robot).
	SlamStartAPIID  int    `json:"slam_start_api_id"`
	SlamStartParams string `json:"slam_start_params"`
	SlamStopAPIID   int    `json:"slam_stop_api_id"`

	// RangeMeters is the half-width (in meters) of the 2D top-down view.
	// The rendered image spans [-RangeMeters, +RangeMeters] in X and Y.
	// Defaults to 10.0.
	RangeMeters float64 `json:"range_meters"`

	// ImageSizePixels is the width and height (in pixels) of the rendered
	// 2D image. Defaults to 512.
	ImageSizePixels int `json:"image_size_pixels"`

	// ZMinMeters / ZMaxMeters filter points by height (meters, in lidar frame)
	// before projecting to 2D. Useful to slice a horizontal band near the
	// sensor. Leave both zero to disable filtering.
	ZMinMeters float64 `json:"z_min_meters"`
	ZMaxMeters float64 `json:"z_max_meters"`
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
	sw     *StringWriter // utlidar switch control; nil if disabled

	slam          *SlamClient // slam_operate control; nil if unavailable
	slamStartID   int64
	slamStartArgs string
	slamStopID    int64

	// 2D rendering params
	rangeMM   float64 // half-width of view, in mm
	imageSize int     // pixels
	zFilter   bool    // whether to apply z filter
	zMinMM    float64
	zMaxMM    float64
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

	rangeMeters := 10.0
	if cfg.RangeMeters > 0 {
		rangeMeters = cfg.RangeMeters
	}
	imageSize := 512
	if cfg.ImageSizePixels > 0 {
		imageSize = cfg.ImageSizePixels
	}
	zFilter := cfg.ZMinMeters != 0 || cfg.ZMaxMeters != 0

	logger.Infof("Initializing G1Lidar (interface=%s topic=%s range=%.2fm size=%dpx)",
		networkInterface, topic, rangeMeters, imageSize)

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	lidar, err := NewLidarClient(topic)
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("lidar subscribe: %w", err)
	}

	// The utlidar only publishes point clouds while it is switched on. Set up a
	// control writer and (unless disabled) turn it on now. A failure here is not
	// fatal: the lidar may already be enabled, or switch control may not apply.
	switchTopic := "rt/utlidar/switch"
	if cfg.SwitchTopic != "" {
		switchTopic = cfg.SwitchTopic
	}
	var sw *StringWriter
	if switchTopic != "-" {
		sw, err = NewStringWriter(switchTopic)
		if err != nil {
			logger.Warnf("G1Lidar: could not create switch writer on %q: %v", switchTopic, err)
		} else if !cfg.DisableSwitchOnStartup {
			logger.Infof("G1Lidar: enabling lidar via %q (ON)", switchTopic)
			if err := sw.Publish("ON"); err != nil {
				logger.Warnf("G1Lidar: failed to publish ON to %q: %v", switchTopic, err)
			}
		}
	}

	// SLAM control: on the G1 the lidar driver only publishes clouds while the
	// SLAM stack is running, so this is the real enable path. Non-fatal if it
	// can't be created.
	slamStartID := int64(ApiSlamStartMapping)
	if cfg.SlamStartAPIID != 0 {
		slamStartID = int64(cfg.SlamStartAPIID)
	}
	slamStartArgs := `{"data":{"slam_type":"indoor"}}`
	if cfg.SlamStartParams != "" {
		slamStartArgs = cfg.SlamStartParams
	}
	slamStopID := int64(ApiSlamCloseSlam)
	if cfg.SlamStopAPIID != 0 {
		slamStopID = int64(cfg.SlamStopAPIID)
	}

	slam, err := NewSlamClient()
	if err != nil {
		logger.Warnf("G1Lidar: could not create slam client: %v", err)
	} else if cfg.StartSlamOnStartup {
		logger.Infof("G1Lidar: starting SLAM (api=%d params=%s)", slamStartID, slamStartArgs)
		if resp, err := slam.Operate(slamStartID, slamStartArgs); err != nil {
			logger.Warnf("G1Lidar: SLAM start failed: %v", err)
		} else {
			logger.Infof("G1Lidar: SLAM start response: %s", resp)
		}
	}

	logger.Info("G1Lidar initialized")

	return &g1Lidar{
		Named:         conf.ResourceName().AsNamed(),
		logger:        logger,
		lidar:         lidar,
		sw:            sw,
		slam:          slam,
		slamStartID:   slamStartID,
		slamStartArgs: slamStartArgs,
		slamStopID:    slamStopID,
		rangeMM:       rangeMeters * 1000.0,
		imageSize:     imageSize,
		zFilter:       zFilter,
		zMinMM:        cfg.ZMinMeters * 1000.0,
		zMaxMM:        cfg.ZMaxMeters * 1000.0,
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

// render2D produces a top-down (bird's-eye) 2D image of the current point cloud.
// X points right, Y points up (screen-up = world +Y), origin at the image center.
// Points are drawn in white; the sensor origin is marked with a red crosshair.
// Point height (Z) is encoded in a blue->green->yellow->red colormap when the
// lidar returns z values, to give a sense of obstacle height.
func (l *g1Lidar) render2D(ctx context.Context) (image.Image, error) {
	pc, err := l.NextPointCloud(ctx)
	if err != nil {
		return nil, err
	}

	size := l.imageSize
	img := image.NewRGBA(image.Rect(0, 0, size, size))
	draw.Draw(img, img.Bounds(), &image.Uniform{C: color.RGBA{R: 16, G: 16, B: 24, A: 255}}, image.Point{}, draw.Src)

	// Draw range rings (at 1/4, 1/2, 3/4 of the range) for visual scale.
	gridColor := color.RGBA{R: 40, G: 40, B: 60, A: 255}
	center := size / 2
	for _, frac := range []float64{0.25, 0.5, 0.75, 1.0} {
		r := float64(size/2) * frac
		drawCircle(img, center, center, int(r), gridColor)
	}
	// Draw axes.
	for i := 0; i < size; i++ {
		img.Set(i, center, gridColor)
		img.Set(center, i, gridColor)
	}

	scale := float64(size) / (2 * l.rangeMM)

	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		if l.zFilter && (p.Z < l.zMinMM || p.Z > l.zMaxMM) {
			return true
		}
		if math.Abs(p.X) > l.rangeMM || math.Abs(p.Y) > l.rangeMM {
			return true
		}
		// World X -> screen X (right), World Y -> screen Y (up, so flip).
		px := center + int(p.X*scale)
		py := center - int(p.Y*scale)
		if px < 0 || px >= size || py < 0 || py >= size {
			return true
		}
		img.Set(px, py, heightColor(p.Z))
		return true
	})

	// Mark sensor origin with a red square.
	origin := color.RGBA{R: 255, G: 64, B: 64, A: 255}
	for dx := -2; dx <= 2; dx++ {
		for dy := -2; dy <= 2; dy++ {
			px, py := center+dx, center+dy
			if px >= 0 && px < size && py >= 0 && py < size {
				img.Set(px, py, origin)
			}
		}
	}
	return img, nil
}

// heightColor maps a z height (mm) to a color. Low = blue, mid = green,
// high = red. Roughly spans -1m .. +2m which covers most indoor obstacles.
func heightColor(zmm float64) color.RGBA {
	const (
		zLo = -1000.0
		zHi = 2000.0
	)
	t := (zmm - zLo) / (zHi - zLo)
	if t < 0 {
		t = 0
	}
	if t > 1 {
		t = 1
	}
	// blue (0,0,255) -> green (0,255,0) -> red (255,0,0)
	var r, g, b uint8
	if t < 0.5 {
		u := t * 2
		r = 0
		g = uint8(255 * u)
		b = uint8(255 * (1 - u))
	} else {
		u := (t - 0.5) * 2
		r = uint8(255 * u)
		g = uint8(255 * (1 - u))
		b = 0
	}
	return color.RGBA{R: r, G: g, B: b, A: 255}
}

// drawCircle draws a (non-filled) circle using the midpoint algorithm.
func drawCircle(img *image.RGBA, cx, cy, r int, c color.Color) {
	if r <= 0 {
		return
	}
	x, y, err := r-1, 0, 0
	dx, dy := 1, 1
	diam := r * 2
	plot := func(px, py int) {
		if px >= img.Bounds().Min.X && px < img.Bounds().Max.X &&
			py >= img.Bounds().Min.Y && py < img.Bounds().Max.Y {
			img.Set(px, py, c)
		}
	}
	for x >= y {
		plot(cx+x, cy+y)
		plot(cx+y, cy+x)
		plot(cx-y, cy+x)
		plot(cx-x, cy+y)
		plot(cx-x, cy-y)
		plot(cx-y, cy-x)
		plot(cx+y, cy-x)
		plot(cx+x, cy-y)

		if err <= 0 {
			y++
			err += dy
			dy += 2
		}
		if err > 0 {
			x--
			dx += 2
			err += dx - diam
		}
	}
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

// --- Camera interface methods: 2D methods render a top-down view of the point cloud. ---

func (l *g1Lidar) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	img, err := l.render2D(ctx)
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}
	var buf bytes.Buffer
	if err := png.Encode(&buf, img); err != nil {
		return nil, camera.ImageMetadata{}, fmt.Errorf("encode png: %w", err)
	}
	return buf.Bytes(), camera.ImageMetadata{MimeType: rdkutils.MimeTypePNG}, nil
}

func (l *g1Lidar) Images(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	img, err := l.render2D(ctx)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	return []camera.NamedImage{
			{Image: img, SourceName: l.Name().ShortName()},
		}, resource.ResponseMetadata{
			CapturedAt: time.Now(),
		}, nil
}

func (l *g1Lidar) Stream(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
	return gostream.NewEmbeddedVideoStreamFromReader(gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
		img, err := l.render2D(ctx)
		if err != nil {
			return nil, nil, err
		}
		return img, func() {}, nil
	})), nil
}

func (l *g1Lidar) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
		MimeTypes:   []string{rdkutils.MimeTypePNG},
		ImageType:   camera.ColorStream,
	}, nil
}

func (l *g1Lidar) Projector(ctx context.Context) (transform.Projector, error) {
	return nil, transform.NewNoIntrinsicsError("")
}

// DoCommand supports manual control of the lidar switch:
//
//	{"command": "lidar_on"}         -> publish "ON" to the switch topic
//	{"command": "lidar_off"}        -> publish "OFF" to the switch topic
//	{"command": "switch", "value": "ON"}  -> publish an arbitrary value
func (l *g1Lidar) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cmdStr, _ := cmd["command"].(string)

	publish := func(value string) (map[string]interface{}, error) {
		if l.sw == nil {
			return map[string]interface{}{"rc": -1.0, "error": "switch control disabled"}, nil
		}
		if err := l.sw.Publish(value); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		l.logger.Infof("G1Lidar: published %q to switch", value)
		return map[string]interface{}{"rc": 0.0}, nil
	}

	switch cmdStr {
	case "dds_scan":
		pubs, err := ListPublications()
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		// Surface it in logs too, since DoCommand output can be awkward to read.
		l.logger.Infof("G1Lidar dds_scan: %d publications discovered", len(pubs))
		for _, p := range pubs {
			l.logger.Infof("  %s", p)
		}
		ifaces := make([]interface{}, len(pubs))
		for i, p := range pubs {
			ifaces[i] = p
		}
		return map[string]interface{}{"rc": 0.0, "count": float64(len(pubs)), "publications": ifaces}, nil
	case "dds_scan_subs":
		subs, err := ListSubscriptions()
		if err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		// Surface anything utlidar-related prominently: it tells us whether the
		// lidar node is alive on the bus even when it publishes no cloud.
		var utlidar []interface{}
		all := make([]interface{}, len(subs))
		for i, s := range subs {
			all[i] = s
			if strings.Contains(s, "utlidar") || strings.Contains(s, "lidar") {
				utlidar = append(utlidar, s)
			}
		}
		l.logger.Infof("G1Lidar dds_scan_subs: %d subscriptions; %d lidar-related", len(subs), len(utlidar))
		for _, s := range utlidar {
			l.logger.Infof("  lidar-related sub: %s", s)
		}
		return map[string]interface{}{
			"rc":            0.0,
			"count":         float64(len(subs)),
			"lidar_related": utlidar,
			"subscriptions": all,
		}, nil
	case "slam_start":
		return l.slamOperate(l.slamStartID, l.slamStartArgs)
	case "slam_stop":
		return l.slamOperate(l.slamStopID, "")
	case "slam_operate":
		apiID, ok := cmd["api_id"].(float64)
		if !ok {
			return map[string]interface{}{"rc": -1.0, "error": "slam_operate requires a numeric 'api_id'"}, nil
		}
		params, _ := cmd["params"].(string)
		return l.slamOperate(int64(apiID), params)
	case "lidar_on":
		return publish("ON")
	case "lidar_off":
		return publish("OFF")
	case "switch":
		value, ok := cmd["value"].(string)
		if !ok {
			return map[string]interface{}{"rc": -1.0, "error": "switch requires a string 'value'"}, nil
		}
		return publish(value)
	}
	return map[string]interface{}{}, nil
}

// slamOperate issues a slam_operate RPC and returns a DoCommand-shaped result.
func (l *g1Lidar) slamOperate(apiID int64, params string) (map[string]interface{}, error) {
	if l.slam == nil {
		return map[string]interface{}{"rc": -1.0, "error": "slam client unavailable"}, nil
	}
	l.logger.Infof("G1Lidar: slam_operate api=%d params=%s", apiID, params)
	resp, err := l.slam.Operate(apiID, params)
	if err != nil {
		return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
	}
	return map[string]interface{}{"rc": 0.0, "response": resp}, nil
}

func (l *g1Lidar) Close(ctx context.Context) error {
	if l.slam != nil {
		l.slam.Close()
	}
	if l.sw != nil {
		l.sw.Close()
	}
	l.lidar.Close()
	ShutdownDDS()
	return nil
}
