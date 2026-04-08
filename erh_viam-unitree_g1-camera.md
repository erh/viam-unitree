# Model erh:viam-unitree:g1-camera

Camera component for the Unitree G1 humanoid robot. Captures JPEG images via the Unitree SDK2 video interface.

## Configuration

```json
{
  "network_interface": "eth0"
}
```

### Attributes

| Name                | Type   | Inclusion | Description                                              |
|---------------------|--------|-----------|----------------------------------------------------------|
| `network_interface` | string | Optional  | Network interface for DDS communication (default: eth0)  |
