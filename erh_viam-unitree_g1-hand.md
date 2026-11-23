# Model erh:viam-unitree:g1-hand

One BrainCo Revo2 hand on the G1, as a gripper. The module talks Modbus RTU to the hand directly, over the Jetson's FTDI USB-to-RS485 adapter (460800 baud), so no Unitree or BrainCo hand service needs to run. Configure one component per hand.

At startup the component reads the hand's SKU and refuses to start if it doesn't match `side`, so a swapped cable can't send one hand's commands to the other.

## Configuration

```json
{
  "side": "right",
  "serial_path": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA0L62G-if01-port0"
}
```

### Attributes

| Name          | Type   | Inclusion | Description                                                        |
|---------------|--------|-----------|--------------------------------------------------------------------|
| `side`        | string | Required  | `left` or `right`; checked against the hand's SKU                  |
| `serial_path` | string | Required  | The hand's RS485 port. Prefer the `/dev/serial/by-id/...` path      |
| `slave_id`    | int    | Optional  | Modbus id (default 126 left, 127 right)                            |
| `grab_pose`   | string | Optional  | Pose `Grab` closes to (default `fist`)                              |

On our G1 the right hand is on FT4232H channel `if01` (`/dev/ttyUSB1`) and the left on `if02` (`/dev/ttyUSB2`).

## Gripper API

- `Open`: opens all fingers and waits until they stop.
- `Grab`: closes to `grab_pose` and returns true if any finger stopped well short of its target (blocked by an object).
- `IsHoldingSomething`: the same blocked check, with per-finger positions and currents in `Meta`.
- `Stop`: holds the fingers where they are.

## DoCommand

Positions are percent closed: 0 = open, 100 = closed. Finger names: `thumb`, `thumb_aux` (thumb rotation), `index`, `middle`, `ring`, `pinky`.

| Command                                   | Description                                         |
|-------------------------------------------|-----------------------------------------------------|
| `{"pose": "point"}`                       | Move to a named pose and wait                       |
| `{"fingers": {"index": 0, "thumb": 50}}`  | Set some fingers; the rest hold their position      |
| `{"get_fingers": true}`                   | Read `percent_closed`, `currents`, `motor_status`   |
| `{"poses": true}`                         | List named poses                                    |

Named poses: `open`, `fist`, `point`, `peace`, `thumbs_up`, `pinch`. `fist` keeps the thumb half closed so it folds over the fingers, matching BrainCo's own close pose.

## Protocol

Register map from BrainCo's [brainco-hand-sdk](https://github.com/BrainCoTech/brainco-hand-sdk) (`tools/canfd-diagnostics`): SKU at holding register 901, status (positions, speeds, currents, motor status) at input registers 2000–2023, firmware at 3000, and six percent targets packed two per register at 1070.
