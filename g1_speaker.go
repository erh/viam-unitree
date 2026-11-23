package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"strconv"
	"time"

	"go.viam.com/rdk/components/audioout"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils"
)

// g1-speaker plays audio through the speaker in the G1's head, using the
// onboard "voice" service (unitree_sdk2 g1/audio/g1_audio_client.hpp).
var g1SpeakerModel = resource.NewModel("erh", "viam-unitree", "g1-speaker")

// Voice service API IDs, from unitree_sdk2's g1_audio_api.hpp.
const (
	ApiAudioStartPlay int64 = 1003
	ApiAudioStopPlay  int64 = 1004
	ApiAudioGetVolume int64 = 1005
	ApiAudioSetVolume int64 = 1006
	ApiAudioSetRGBLed int64 = 1010
)

const (
	// The voice service plays 16 kHz mono 16-bit PCM only.
	g1SpeakerSampleRate  = 16000
	g1SpeakerBytesPerSec = g1SpeakerSampleRate * 2
	// Unitree's example sends 3 s of audio per PlayStream call, one second
	// apart; the robot buffers chunks that share a stream_id.
	g1SpeakerChunkBytes  = 3 * g1SpeakerBytesPerSec
	g1SpeakerChunkPacing = time.Second
	g1SpeakerAppName     = "viam"
	g1SpeakerRPCTimeout  = 2000
)

type g1SpeakerConfig struct {
	NetworkInterface string `json:"network_interface"`
	Volume           *int   `json:"volume,omitempty"`
}

func (c *g1SpeakerConfig) Validate(path string) ([]string, []string, error) {
	if c.Volume != nil && (*c.Volume < 0 || *c.Volume > 100) {
		return nil, nil, fmt.Errorf("%s: volume must be between 0 and 100", path)
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(audioout.API, g1SpeakerModel,
		resource.Registration[audioout.AudioOut, *g1SpeakerConfig]{
			Constructor: newG1Speaker,
		})
}

type g1Speaker struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	rpc    *RPCClient
}

func newG1Speaker(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (audioout.AudioOut, error) {
	cfg, err := resource.NativeConfig[*g1SpeakerConfig](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}
	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	rpc, err := NewRPCClient("voice", true)
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("voice client: %w", err)
	}

	s := &g1Speaker{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		rpc:    rpc,
	}
	if cfg.Volume != nil {
		if err := s.setVolume(*cfg.Volume); err != nil {
			logger.Warnf("g1-speaker: could not set initial volume: %v", err)
		}
	}
	return s, nil
}

func (s *g1Speaker) Properties(ctx context.Context, extra map[string]interface{}) (utils.Properties, error) {
	return utils.Properties{
		SupportedCodecs: []string{utils.CodecPCM16},
		SampleRateHz:    g1SpeakerSampleRate,
		NumChannels:     1,
	}, nil
}

func checkG1SpeakerFormat(info *utils.AudioInfo) error {
	if info == nil {
		return nil
	}
	if info.Codec != utils.CodecPCM16 || info.SampleRateHz != g1SpeakerSampleRate || info.NumChannels != 1 {
		return fmt.Errorf("g1-speaker plays pcm16 at %d Hz mono only, got %s at %d Hz with %d channels",
			g1SpeakerSampleRate, info.Codec, info.SampleRateHz, info.NumChannels)
	}
	return nil
}

// Play sends the audio in chunks and blocks until the robot has played it.
func (s *g1Speaker) Play(ctx context.Context, data []byte, info *utils.AudioInfo, extra map[string]interface{}) error {
	if err := checkG1SpeakerFormat(info); err != nil {
		return err
	}
	streamID := newG1StreamID()
	start := time.Now()
	for offset := 0; offset < len(data); offset += g1SpeakerChunkBytes {
		chunk := data[offset:min(offset+g1SpeakerChunkBytes, len(data))]
		if err := s.sendChunk(ctx, streamID, chunk); err != nil {
			return err
		}
	}
	return s.waitForPlayback(ctx, len(data), start)
}

// PlayStream regroups incoming chunks into the voice service's chunk size
// under one stream_id, then blocks until the robot has played everything.
func (s *g1Speaker) PlayStream(ctx context.Context, info *utils.AudioInfo, chunks <-chan []byte, extra map[string]interface{}) error {
	if err := checkG1SpeakerFormat(info); err != nil {
		return err
	}
	streamID := newG1StreamID()
	start := time.Now()
	total := 0
	var pending []byte
	for {
		select {
		case <-ctx.Done():
			return s.stopAfter(ctx.Err())
		case chunk, ok := <-chunks:
			if !ok {
				if len(pending) > 0 {
					if err := s.sendChunk(ctx, streamID, pending); err != nil {
						return err
					}
				}
				return s.waitForPlayback(ctx, total, start)
			}
			total += len(chunk)
			pending = append(pending, chunk...)
			for len(pending) >= g1SpeakerChunkBytes {
				if err := s.sendChunk(ctx, streamID, pending[:g1SpeakerChunkBytes]); err != nil {
					return err
				}
				pending = pending[g1SpeakerChunkBytes:]
			}
		}
	}
}

func newG1StreamID() string {
	return strconv.FormatInt(time.Now().UnixNano(), 10)
}

func (s *g1Speaker) sendChunk(ctx context.Context, streamID string, chunk []byte) error {
	params, err := json.Marshal(map[string]string{"app_name": g1SpeakerAppName, "stream_id": streamID})
	if err != nil {
		return err
	}
	if _, _, err := s.rpc.CallBinary(ApiAudioStartPlay, string(params), chunk, g1SpeakerRPCTimeout); err != nil {
		return fmt.Errorf("play audio: %w", err)
	}
	return s.stopAfter(sleepContext(ctx, g1SpeakerChunkPacing))
}

// waitForPlayback blocks until byteCount bytes of audio, sent since start,
// have had time to play.
func (s *g1Speaker) waitForPlayback(ctx context.Context, byteCount int, start time.Time) error {
	duration := time.Duration(byteCount) * time.Second / g1SpeakerBytesPerSec
	return s.stopAfter(sleepContext(ctx, duration-time.Since(start)))
}

// stopAfter silences the speaker when err is non-nil (e.g. the caller
// cancelled), then returns err.
func (s *g1Speaker) stopAfter(err error) error {
	if err != nil {
		if stopErr := s.stop(); stopErr != nil {
			s.logger.Warnf("g1-speaker: stop after error: %v", stopErr)
		}
	}
	return err
}

func sleepContext(ctx context.Context, d time.Duration) error {
	if d <= 0 {
		return ctx.Err()
	}
	timer := time.NewTimer(d)
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return nil
	}
}

func (s *g1Speaker) stop() error {
	params, err := json.Marshal(map[string]string{"app_name": g1SpeakerAppName})
	if err != nil {
		return err
	}
	_, _, err = s.rpc.Call(ApiAudioStopPlay, string(params), g1SpeakerRPCTimeout)
	return err
}

func (s *g1Speaker) setVolume(volume int) error {
	if volume < 0 || volume > 100 {
		return fmt.Errorf("volume must be between 0 and 100, got %d", volume)
	}
	_, _, err := s.rpc.Call(ApiAudioSetVolume, fmt.Sprintf(`{"volume":%d}`, volume), g1SpeakerRPCTimeout)
	return err
}

func (s *g1Speaker) getVolume() (int, error) {
	data, _, err := s.rpc.Call(ApiAudioGetVolume, "{}", g1SpeakerRPCTimeout)
	if err != nil {
		return 0, err
	}
	var resp struct {
		Volume int `json:"volume"`
	}
	if err := json.Unmarshal([]byte(data), &resp); err != nil {
		return 0, fmt.Errorf("parse volume %q: %w", data, err)
	}
	return resp.Volume, nil
}

func (s *g1Speaker) setLED(r, g, b int) error {
	for _, v := range []int{r, g, b} {
		if v < 0 || v > 255 {
			return fmt.Errorf("led values must be between 0 and 255, got %d", v)
		}
	}
	_, _, err := s.rpc.Call(ApiAudioSetRGBLed, fmt.Sprintf(`{"R":%d,"G":%d,"B":%d}`, r, g, b), g1SpeakerRPCTimeout)
	return err
}

// DoCommand:
//
//	{"set_volume": 75}      -> {"volume": 75}
//	{"get_volume": true}    -> {"volume": 75}
//	{"stop": true}          -> {"stopped": true}
//	{"led": [255, 0, 255]}  -> {"led": [255, 0, 255]}  (head light color)
func (s *g1Speaker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if raw, ok := cmd["set_volume"]; ok {
		volume, err := numericToInt(raw)
		if err != nil {
			return nil, fmt.Errorf("set_volume: %w", err)
		}
		if err := s.setVolume(volume); err != nil {
			return nil, err
		}
		return map[string]interface{}{"volume": volume}, nil
	}
	if _, ok := cmd["get_volume"]; ok {
		volume, err := s.getVolume()
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"volume": volume}, nil
	}
	if _, ok := cmd["stop"]; ok {
		if err := s.stop(); err != nil {
			return nil, err
		}
		return map[string]interface{}{"stopped": true}, nil
	}
	if raw, ok := cmd["led"].([]interface{}); ok {
		if len(raw) != 3 {
			return nil, errors.New("led takes [r, g, b]")
		}
		rgb := make([]int, 3)
		for i, v := range raw {
			n, err := numericToInt(v)
			if err != nil {
				return nil, fmt.Errorf("led: %w", err)
			}
			rgb[i] = n
		}
		if err := s.setLED(rgb[0], rgb[1], rgb[2]); err != nil {
			return nil, err
		}
		return map[string]interface{}{"led": raw}, nil
	}
	return nil, fmt.Errorf("unknown command; supported: set_volume, get_volume, stop, led")
}

func (s *g1Speaker) Close(ctx context.Context) error {
	if s.rpc != nil {
		s.rpc.Close()
		s.rpc = nil
	}
	ShutdownDDS()
	return nil
}
