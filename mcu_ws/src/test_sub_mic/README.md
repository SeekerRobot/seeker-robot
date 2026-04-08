# test_sub_mic

Tests the onboard PDM microphone on the Seeed XIAO ESP32-S3 Sense via the
`MicSubsystem` abstraction. Streams raw 16-bit PCM audio over HTTP on port 81
— no micro-ROS required.

## Endpoints

| Route | Description |
|---|---|
| `GET /stream` | Browser player (Web Audio API, ~100ms latency) |
| `GET /audio` | Raw 16-bit signed PCM, 16kHz, mono — streamed continuously |

Only one client can consume `/audio` at a time. A second request returns
`HTTP 500 Stream busy`.

## Usage

### 1. Find the device IP

Open a serial monitor at **921600 baud** after flashing. The static IP is set
in `mcu_ws/platformio/network_config.ini`.

### 2. Browser stream

Navigate to `http://<device-ip>:81/stream` and click **Start Listening**.
Status should change to `Streaming` and you should hear audio through your
speakers.

### 3. Raw audio via curl

**Capture to file:**
```bash
curl -s http://<device-ip>:81/audio --output audio.raw
# Ctrl+C after a few seconds, then play back:
aplay -r 16000 -f S16_LE -c 1 audio.raw
# Or convert to WAV:
sox -r 16000 -e signed -b 16 -c 1 audio.raw output.wav
```

**Pipe directly to speaker:**
```bash
curl -s http://<device-ip>:81/audio | aplay -r 16000 -f S16_LE -c 1
```

**Verify the mic is picking up sound (values should be non-zero and varying):**
```bash
curl -s http://<device-ip>:81/audio | od -t d2 -w16 | head -20
```

### Expected serial output

```
[Mic] I2S init OK (16000 Hz, CLK=42, DATA=41)
[Mic] Audio stream at http://<ip>:81/stream
[Mic] Audio stream connected        ← when a client connects
[Mic] Audio stream disconnected     ← when the client disconnects
[Mic] Stream up on port 81          ← periodic status (every 5 s)
```

## See also

- `test_raw_mic` — same hardware, direct I2S without the subsystem
- `test_sub_cam_mic` — camera + mic running side by side
