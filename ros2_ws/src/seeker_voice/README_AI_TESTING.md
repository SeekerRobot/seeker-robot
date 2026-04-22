# Seeker AI — Autonomous Object Search Integration Guide

This guide describes how to test the **Gemini-powered Object Seeker** pipeline. It explains how to verify that the robot can take a natural language request for *any* object, map it to a known category using the LLM, and trigger the autonomous search behavior in the simulation.

---

## The "Find" Architecture (Brain to Body Action)

Unlike simple movements (which use topics), the **Find** command triggers a **ROS 2 Action** (`SeekObject`). This allows for a persistent, long-running mission with feedback.

1.  **Ingestion**: You type a command into `voice_terminal.py` (e.g., `"hey hatsune find the coke over"`).
2.  **Mapping (Gemini)**: The `command_node` sends the text to Gemini. Gemini is instructed to map the user's object to the closest category in the **80 COCO classes** (e.g., `Coke` → `bottle`, `Red Ball` → `sports ball`).
3.  **The Goal**: The `command_node` (Action Client) sends a `SeekObject` goal to the `ball_searcher` (Action Server).
4.  **The Search**: The `ball_searcher` in `seeker_navigation` starts the physical search:
    *   **Phase 1**: 360° rotation to scan the immediate area.
    *   **Phase 2**: Frontier exploration (moving to unknown parts of the map).
    *   **Phase 3**: Final approach once the object is spotted by the camera.

---

## Dynamic Miku Voice Replies

Every accepted voice command triggers a unique, in-character Hatsune Miku
reply through the Fish Audio TTS pipeline — no canned strings. The prompt
sent to Gemini asks it to emit a short, energetic acknowledgment in the
`response_phrase` field of the structured output (under 12 words, varied
each call, references the action and — for `find` — the target object).
`command_node` uses that phrase twice: once as the confirmation prompt
(`"{response_phrase} Is that right?"`) and again as execution feedback
after the user confirms. Example parses:

| User says | Gemini returns | Robot says (confirmation) |
|---|---|---|
| `hey hatsune spin around for me over` | `action=spin, response_phrase="Hehe, spinning around for you~!"` | *"Hehe, spinning around for you~! Is that right?"* |
| `hey hatsune go find the coke over` | `action=find, target=bottle, response_phrase="Searching for that bottle, yay~!"` | *"Searching for that bottle, yay~! Is that right?"* |

**Default model**: `gemini-2.5-flash` (overridable via the `gemini_model`
node parameter). `gemini-2.0-flash` is no longer available to new users.

**Offline fallback**: if Gemini is unreachable or quota-limited,
`command_node` falls back to keyword matching and uses the static
`response_phrase` baked into the fallback `Output(...)` construction.

### Quick isolated test (no sim, no mic)

Start just the two nodes and inject a fake transcription:

```bash
# Terminal A — TTS bridge
ros2 run seeker_tts tts_node

# Terminal B — Brain
ros2 run seeker_voice command_node

# Terminal C — audition audio on the HOST (not in container)
./ros2_ws/src/seeker_tts/tools/listen_audio.sh

# Terminal D — fake a transcription (note the {...} YAML dict form)
ros2 topic pub --once /audio_transcription std_msgs/String \
  "{data: 'hey hatsune spin around for me over'}"
```

Expected: a dynamic Miku confirmation plays out of the host speaker within
~2 seconds. The `command_node` log will show
`Gemini parsed: action=RobotAction.SPIN, target=none` and the `tts_node`
log will show `TTS request: <the dynamic phrase>` + a PCM byte count.

---

## Prerequisites

1.  **API Key**: A valid `GEMINI_API_KEY` must be in your `docker/.env`.
2.  **Build**: You must build the navigation stack:
    ```bash
    colcon build --packages-select mcu_msgs seeker_voice seeker_navigation seeker_gazebo seeker_sim seeker_vision
    source install/setup.bash
    ```

---

## Detailed Test Procedure

You will need three terminals inside the Docker container.

### Terminal 1: The Entire Stack (Simulation + Nav2)
Launch the full autonomy simulation. This starts Gazebo, the SLAM mapping, and the Nav2 path planner.
```bash
ros2 launch seeker_gazebo sim_ball_search.launch.py
```
*Note: This launch takes ~25 seconds to fully initialize the lifecycle managers. Wait until you see `ball_searcher` starting in the logs.*

### Terminal 2: The Brain (Gemini Node)
Launch the AI command processor.
```bash
ros2 launch seeker_voice local_mic.launch.py
```

### Terminal 3: The Input (Voice Terminal)
Run the script to type your commands.
```bash
python3 ~/scripts/voice_terminal.py
```

---

## Testing "Any Object" Mapping

The power of Gemini is mapping fuzzy user input to precise robot categories. Try these commands in **Terminal 3**:

| User Types | Gemini Mapping | Target Object Category |
|---|---|---|
| `hey hatsune find the coke over` | `coke` → `bottle` | `bottle` |
| `hey hatsune look for a kitty cat over` | `kitty cat` → `cat` | `cat` |
| `hey hatsune go find the red ball over` | `red ball` → `sports ball` | `sports ball` |
| `hey hatsune i lost my teddy over` | `teddy` → `teddy bear` | `teddy bear` |

---

## How to Verify Registration

You can confirm the robot has "registered" the goal without even looking at the Gazebo window:

### 1. Watch the Brain's Log
In **Terminal 2**, you should see Gemini's classification:
```text
[command_node]: Gemini parsed: action=find the object, target=bottle
[command_node]: Sending SeekObject goal for 'bottle'...
```

### 2. Verify the Action Goal (Terminal 4 / New Terminal)
Run this command to see the goal being sent to the navigation stack:
```bash
ros2 topic echo /seek_object/_action/goal
```
You will see the `target_object` field filled with the category Gemini chose.

### 3. Visual Confirmation (Gazebo)
*   **Rotation**: As soon as the command is registered, the hexapod in Gazebo will start **rotating in place** (this is the "seed map" phase).
*   **Log Output**: The `ball_searcher` will log: `[ball_searcher]: New goal received: Find a bottle`.

### 4. Nav2 Status
If the robot starts moving toward a frontier (a "black" area on the map), it means the Nav2 stack is successfully executing the AI's request.

---

## Troubleshooting the "Any Object" Mapping
If Gemini fails to map correctly:
1.  **Check the logs**: If it says `NONE OF THE ABOVE`, Gemini didn't understand it was a "Find" request.
2.  **Prompt Instruction**: The logic for mapping is defined in `seeker_voice/command_node.py` under `_INSTRUCTION`. It contains a list of the 80 allowed categories.
3.  **Offline Mode**: If Gemini is offline, the system falls back to simple keyword matching (which is much less flexible than the "any object" mapping).
