# Model erh:viam-unitree:g1-camera

Camera component for the Unitree G1 humanoid robot's head camera.

The G1's head camera is an Intel RealSense attached by USB to the onboard
development computer (PC2). By default this model reads its color stream
directly as a V4L2 device, so **the module must run on the computer the camera
is plugged into**.

Older versions used the Unitree `videohub` DDS service. That is a Go2 API that
stock G1 firmware does not serve, so every frame request timed out. It is still
available with `"source": "videohub"`.

## Configuration

```json
{}
```

With no `video_path`, the model tries each `/dev/videoN` whose V4L2 name
contains "RealSense", then any device, and uses the first that supports
`format` (default `YUY2`, which only the RealSense color node offers, so the
depth and infrared nodes are skipped).

### Attributes

| Name                | Type   | Inclusion | Description |
|---------------------|--------|-----------|-------------|
| `source`            | string | Optional  | `v4l2` (default) or `videohub` |
| `video_path`        | string | Optional  | V4L2 device to use, e.g. `/dev/video4` (default: auto-detect) |
| `format`            | string | Optional  | Pixel format, as in the RDK webcam model (default: `YUY2`) |
| `width_px`          | int    | Optional  | Requested width (default: ~640) |
| `height_px`         | int    | Optional  | Requested height (default: ~480) |
| `frame_rate`        | float  | Optional  | Requested frame rate (default: ~30) |
| `network_interface` | string | Optional  | `videohub` only: DDS network interface (default: `eth0`) |

To find the color node by hand on the robot:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video4 --list-formats
```
