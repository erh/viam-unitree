# Model erh:viam-unitree:g1-lidar

Point-cloud camera for the Unitree G1 head lidar (Livox Mid-360).

## Backends

| `source` | Behavior |
|----------|----------|
| `livox` (default) | Talks to the Mid-360 over UDP via [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2). Use this when `/unitree/module/` has no lidar package and Unitree DDS cloud topics are absent. |
| `dds` | Subscribe to a Unitree `PointCloud2` DDS topic (e.g. `rt/utlidar/cloud` or `rt/utlidar/cloud_livox_mid360`). |

## Configuration (Livox — default)

```json
{
  "source": "livox",
  "livox_host_ip": "192.168.123.164",
  "livox_lidar_ip": "192.168.123.120",
  "livox_model": "mid360",
  "network_interface": "eth0"
}
```

PC2 must be on `192.168.123.0/24` and able to `ping 192.168.123.120`. Stop any other Livox ROS driver that binds the same host ports.

| Attribute | Default | Notes |
|-----------|---------|-------|
| `source` | `livox` | `livox` or `dds` |
| `livox_host_ip` | `192.168.123.164` | Address Mid-360 sends clouds to (PC2) |
| `livox_lidar_ip` | `192.168.123.120` | Sensor IP (informational for generated config) |
| `livox_model` | `mid360` | Use `mid360s` for post–April 2026 Mid360s units |
| `livox_config_path` | (generated) | Optional path to a Livox-SDK2 JSON config |
| `livox_frame_ms` | `100` | Frame assembly window (~10 Hz) |
| `disable_invert_mount` | `false` | G1 head mount is upside down; correction `(x,-y,-z)` is on by default |
| `network_interface` | `eth0` | Also used to init DDS for `dds_scan` / slam DoCommands |

## Configuration (DDS)

```json
{
  "source": "dds",
  "topic": "rt/utlidar/cloud_livox_mid360",
  "network_interface": "eth0",
  "start_slam_on_startup": true
}
```

Requires Unitree’s onboard lidar/SLAM services to actually publish the topic (`dds_scan` should list it).
