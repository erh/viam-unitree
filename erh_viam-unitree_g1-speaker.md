# Model erh:viam-unitree:g1-speaker

Audio out through the speaker in the G1's head. Talks to the robot's onboard `voice` service, the same one unitree_sdk2's `g1::AudioClient` uses, so no extra hardware or system audio setup is needed.

## Configuration

```json
{
  "network_interface": "eth0",
  "volume": 80
}
```

### Attributes

| Name                | Type   | Inclusion | Description                                                  |
|---------------------|--------|-----------|--------------------------------------------------------------|
| `network_interface` | string | Optional  | Network interface for DDS (default: eth0)                    |
| `volume`            | int    | Optional  | Volume (0–100) to set when the component starts              |

## Audio format

The voice service plays 16 kHz, mono, 16-bit PCM only (`pcm16`). `Properties` reports this, and `Play` / `PlayStream` reject other formats, so resample before sending (for example, configure Piper TTS for 16 kHz output).

`Play` and `PlayStream` block until the robot has finished playing. Cancelling the call stops playback.

## DoCommand

| Command                   | Returns                  | Description                    |
|---------------------------|--------------------------|--------------------------------|
| `{"set_volume": 75}`      | `{"volume": 75}`         | Set volume, 0–100              |
| `{"get_volume": true}`    | `{"volume": 75}`         | Read the current volume        |
| `{"stop": true}`          | `{"stopped": true}`      | Stop playback immediately      |
| `{"led": [255, 0, 255]}`  | `{"led": [255, 0, 255]}` | Set the head light color (RGB) |
