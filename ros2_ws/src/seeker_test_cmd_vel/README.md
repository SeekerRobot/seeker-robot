# Seeker AI — Simulation Integration Guide

This guide describes the "Brain-to-Body" testing pipeline. It explains how to verify the entire stack: from typing a natural language command in a terminal, through **Gemini LLM** intent classification, to the hexapod physically walking in the **Gazebo Harmonic** simulation.

---

## The Information Pipeline

The system follows a reactive, asynchronous message flow across four distinct stages:

1.  **Ingestion**: You type a command into `voice_terminal.py`. This bypasses the Whisper microphone node and injects text directly into the system.
2.  **Classification (The Brain)**: The `command_node` (Package: `seeker_voice`) receives the text. If it detects the wake word (**"hey hatsune"**), it sends the snippet to the **Google Gemini API**. Gemini returns a structured JSON intent (e.g., `move forward`).
3.  **Translation (The Muscle)**: The `velocity_node` (Package: `seeker_test_cmd`) subscribes to the AI's intent. it translates strings like `"dance"` or `"move forward"` into standard ROS 2 `geometry_msgs/Twist` velocity commands.
4.  **Simulation (The Body)**: 
    *   `fake_mcu_node` (Package: `seeker_sim`) converts the Velocity into 12 individual joint position commands.
    *   `ros_gz_bridge` forwards these positions to Gazebo's physics engine.

---

## Prerequisites

1.  **API Key**: You must have a valid `GEMINI_API_KEY` set in your `docker/.env` file.
2.  **Build**: Ensure all packages are built inside the container:
    ```bash
    colcon build --packages-select mcu_msgs seeker_voice seeker_test_cmd seeker_gazebo seeker_sim seeker_description
    source install/setup.bash
    ```

---

## Detailed Test Procedure

To test the full loop, you will need four terminal sessions inside the Docker container.

### Terminal 1: The World (Gazebo)
Launch the simulation environment. This brings up the physics engine and the joint-level controllers.
```bash
ros2 launch seeker_gazebo sim_teleop.launch.py
```
*Wait until the Gazebo GUI (or server) is fully loaded and you see the hexapod standing.*

### Terminal 2: The Brain (Gemini Node)
Launch the AI command processor. We use `local_mic.launch.py` because it starts the `command_node` with the correct Gemini parameters, even though we will bypass the actual microphone.
```bash
ros2 launch seeker_voice local_mic.launch.py
```

### Terminal 3: The Translator (Velocity Node)
Start the node that turns AI intents into movement.
```bash
ros2 run seeker_test_cmd velocity_node
```

### Terminal 4: The Input (Voice Terminal)
Run the script that lets you "fake" a voice transcription by typing.
```bash
python3 ~/scripts/voice_terminal.py
```

---

## Running a Command

In **Terminal 4**, type the following exactly (including the wake and end words):

> `hey hatsune move forward over`

### What happens behind the scenes:
1.  **Terminal 4** publishes `hey hatsune move forward over` to `/audio_transcription`.
2.  **Terminal 2** (`command_node`) sees the wake word, calls the Gemini API, and receives the classification: `move forward`. It then publishes `move forward` to `/voice_command`.
3.  **Terminal 3** (`velocity_node`) receives `move forward` and publishes a `Twist` message with `linear.x = 0.5` to `/cmd_vel`.
4.  **Terminal 1** (`fake_mcu_node`) receives the `Twist` and starts oscillating the leg joints in a tripod gait.

---

## How to Verify "Registration on Gazebo"

Detailed verification is done by watching the message hand-off at the bridge:

### 1. Check the ROS ↔ Gazebo Bridge
Look at the output of **Terminal 1**. When the robot moves, you should see:
```text
[ros_gz_bridge]: Passing message from ROS std_msgs/msg/Float64 to Gazebo gz.msgs.Double
```
If you see this, ROS has successfully handed the AI's movement intent to the Gazebo physics engine.

### 2. Verify the Velocity Topic
In a new terminal, run:
```bash
ros2 topic echo /cmd_vel
```
When you send the command, you should see a non-zero `linear` or `angular` value appear here for 15 seconds (the default safety duration).

### 3. Debugging the AI Classification
If the robot doesn't move, check the logs in **Terminal 2**. 
*   **Success**: `[command_node]: Gemini parsed: action=move forward`
*   **API Error**: `[command_node]: Gemini error: ...` (usually indicates an expired API key or network issue).
*   **Heuristic Fallback**: If Gemini is offline, the node will log that it is using "heuristic matching" to try and guess the command locally.

---

## Supported Commands
The `velocity_node` in `seeker_test_cmd` currently translates the following intents:
*   `move forward`
*   `move backward`
*   `move left` / `move right`
*   `spin`
*   `dance`
*   `stop` (Emergency stop)
