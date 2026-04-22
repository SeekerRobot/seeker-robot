# seeker_tts

ROS 2 node that bridges Fish Audio TTS and pre-rendered WAV files to a raw
PCM HTTP stream consumed by the ESP32 speaker firmware.

## Architecture

```
/audio_tts_input  (std_msgs/String)  →  Fish Audio API  →┐
                                                          ├→  queue  →  :8383/audio_out  →  ESP32
/audio_play_file  (std_msgs/String)  →  WAV loader     →┘
```

- **`/audio_tts_input`** — publish text; the node calls Fish Audio and streams
  the result. Higher priority (blocking put).
- **`/audio_play_file`** — publish an absolute WAV path; the node loads and
  queues it. Lower priority (dropped if TTS is already streaming).
- **`:8383/audio_out`** — raw 16-bit mono PCM at 16 kHz, one clip per HTTP
  connection. Returns 204 when idle so the ESP32 reconnects immediately.

## Changes made (integration-reyjay branch)

### Voice persona: Hatsune Miku
All speech phrases use a Miku voice clone via `FISH_REFERENCE_ID`. Phrases
are written to match her energetic, upbeat character.

### State-transition audio hooks (seeker_navigation)
`object_seeker.py` now publishes audio cues on every state change during a
`SeekObject` action. Logic lives in `seeker_navigation/search_voice.py`.

| Event | Audio | Type |
|---|---|---|
| Seek goal accepted | "Let's find it! Here we go!" | WAV |
| Object spotted | "I see it! I see it!" | WAV |
| Object reached | "Yes! I made it!" | WAV |
| Search failed / timeout | "Aww... I couldn't find it" | WAV |
| Search canceled | "Oh okay, stopping now!" | WAV |
| Periodic chime (~every 30 s) | "Still searching... la la la..." | WAV |
| Periodic variety (~25% of chimes) | random phrase | Fish TTS (live) |
| Frontier → coverage | "Let me try a wider sweep!" | Fish TTS (live) |

### Static WAVs (`sounds/`)
Pre-rendered once via `tools/generate_sounds.py`. Playback requires no API
call so latency is instant. Re-generate only when phrases change.

### Launch integration
`sim_object_seek.launch.py` now starts `tts_node` automatically at `t=0`.

---

## Setup

### 1. API keys
Keys are read from `docker/.env` (auto-detected by the generate script) or
from environment variables. Ensure `docker/.env` contains:

```
FISH_API_KEY=<your key from https://fish.audio>
FISH_REFERENCE_ID=<Miku voice clone reference ID>
```

These are forwarded into the container by docker-compose.

### 2. Generate static WAVs (run once)

```bash
# Inside container
python3 ~/ros2_workspaces/src/seeker_ros/seeker_tts/tools/generate_sounds.py
```

To regenerate a single cue:
```bash
python3 ~/ros2_workspaces/src/seeker_ros/seeker_tts/tools/generate_sounds.py --only searching
```

Available events: `search_start`, `object_spotted`, `object_reached`,
`search_failed`, `search_canceled`, `searching`

### 3. Build

```bash
cd ~/ros2_workspaces
colcon build --packages-select seeker_tts seeker_navigation seeker_gazebo
source install/setup.bash
```

---

## Testing in Gazebo

### Launch the full sim (includes tts_node)
```bash
ros2 launch seeker_gazebo sim_object_seek.launch.py
```

### Verify tts_node is running
```bash
ros2 node list | grep tts
ros2 topic list | grep audio
```

### Trigger a seek
```bash
ros2 run seeker_navigation find sports_ball --timeout 120 --feedback
```

### Quick topic tests (no sim needed)
```bash
# Test WAV playback (instant, no API key needed)
ros2 topic pub --once /audio_play_file std_msgs/String \
  "data: '/home/ubuntu/ros2_workspaces/install/seeker_tts/share/seeker_tts/sounds/search_start.wav'"

# Test live TTS (requires FISH_API_KEY in container env)
ros2 topic pub --once /audio_tts_input std_msgs/String "data: 'Hello! I am Miku!'"
```

### Audition audio on the host machine

> **Important:** run this on the **host** (`jay@code`), NOT inside the
> container (`ubuntu@code`). The container has no audio hardware. Because
> the container uses `network_mode: host`, port 8383 is directly on
> `localhost` from the host machine.

```bash
# Install ffmpeg on host if needed
sudo apt install -y ffmpeg

# Listen loop — run from HOST terminal only
./ros2_ws/src/seeker_tts/tools/listen_audio.sh
```

### On the real robot
No extra steps — the ESP32 firmware already polls `:8383/audio_out`
continuously. Flash the firmware, ensure it can reach the ROS host IP on
port 8383, and audio plays through the physical speaker automatically.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `fish_api_key` | `""` | Fish Audio API key (falls back to `FISH_API_KEY` env var) |
| `fish_reference_id` | `""` | Voice clone reference ID (falls back to `FISH_REFERENCE_ID` env var) |
| `fish_model` | `s2-pro` | Fish Audio model |
| `serve_port` | `8383` | HTTP server port |
| `sample_rate` | `16000` | Output sample rate (Hz) |
| `eq_bass_hz` | `300.0` | Low-shelf EQ corner frequency |
| `eq_bass_db` | `0.0` | Low-shelf EQ gain (dB); 0 = bypass |
