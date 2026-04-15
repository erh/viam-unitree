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

	logger.Info("G1Lidar initialized")

	return &g1Lidar{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		lidar:     lidar,
		rangeMM:   rangeMeters * 1000.0,
		imageSize: imageSize,
		zFilter:   zFilter,
		zMinMM:    cfg.ZMinMeters * 1000.0,
		zMaxMM:    cfg.ZMaxMeters * 1000.0,
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

func (l *g1Lidar) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (l *g1Lidar) Close(ctx context.Context) error {
	l.lidar.Close()
	ShutdownDDS()
	return nil
}
