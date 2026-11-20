# Model erh:viam-unitree:g1-depth

Depth images and point clouds from the G1 head camera (Intel RealSense D435i). It reads the camera's 16-bit depth (Z16) V4L2 node directly, so it needs no librealsense. The depth node is separate from the color node `g1-camera` uses, so both can run together.

## Configuration

```json
{}
```

All attributes are optional:

| Name             | Type   | Default | Description                                                       |
|------------------|--------|---------|-------------------------------------------------------------------|
| `video_path`     | string | auto    | Depth node, e.g. `/dev/video0`. Default finds the RealSense node that offers Z16 |
| `width`, `height`| int    | 640, 480 | Depth image size; set both or neither                            |
| `fx`, `fy`, `ppx`, `ppy` | float | nominal D435i | Intrinsics in pixels. Default is fx = fy = 383 at 640x480 (scaled by height) with the principal point at the centre |
| `depth_scale_mm` | float  | 1       | Millimetres per depth unit                                        |
| `min_depth_mm`, `max_depth_mm` | float | 200, 5000 | Points outside this range are dropped                   |
| `stride`         | int    | 4       | Keep every Nth pixel in each direction for point clouds           |

Without librealsense the factory calibration can't be read, so the default intrinsics are a nominal model. They are typically within a few percent, which is fine for obstacle avoidance; set them for anything more precise.

## Output

- `NextPointCloud`: points in millimetres in the camera-optical frame (+Z forward, +X right, +Y down). Give the component a frame whose orientation aims +Z along the robot's forward axis.
- `Images`: one depth image (`image/vnd.viam.dep`).
- `DoCommand {"status": true}`: device, size, intrinsics and age of the latest frame.

Frames older than 1 s are not served, so a stalled stream shows up as an error instead of a frozen cloud.
