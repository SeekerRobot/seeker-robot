# seeker_gazebo — Simulation Runbook

Gazebo Harmonic simulation for the Seeker hexapod. Launches everything needed
to drive the robot with `teleop_twist_keyboard` and see the legs actually walk.

---

## Prerequisites

- Docker container running (`dev` build target — includes Gazebo Harmonic)
- Packages built:
  ```bash
  cd ~/ros2_workspaces
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select mcu_msgs seeker_description seeker_gazebo seeker_sim
  source install/setup.bash
  ```

---

## Quick Start

```bash
# Terminal 1 — launch simulation
ros2 launch seeker_gazebo sim_teleop.launch.py

# Terminal 2 — drive the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Teleop keys:

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `k` | Stop (clean landing) |
| `q` / `z` | Increase / decrease linear speed |
| `w` / `x` | Increase / decrease angular speed |

---

## What You Should See

1. Gazebo opens with a walled test world containing the hexapod
2. When you press `i`, the hexapod walks with alternating tripod legs
   - Group A (FL, MR, RL) and Group B (FR, ML, RR) alternate in stance/flight
3. The TF tree shows 12 articulated leg joints
4. `/mcu/imu` and `/mcu/scan` are live even without hardware

---

## Verify Topics

Run these from the **same terminal** that launched the simulation (or a terminal
that shares the same shell session). Due to a DDS multicast isolation quirk in
Docker, `ros2 topic hz` run in a *separate* `docker exec` session may report
topics as unpublished even when they are flowing correctly — this is a testing
artefact, not a real failure.

```bash
# In the same terminal, or after: source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic hz /mcu/joint_states     # ~100 Hz
ros2 topic hz /mcu/imu              # ~200 Hz
ros2 topic hz /mcu/scan             # ~10 Hz
ros2 topic echo /cmd_vel            # shows Twist when teleop key held
ros2 run tf2_tools view_frames      # view full TF tree as PDF
```

To confirm joint commands reach Gazebo without opening RViz, check the bridge log:
```
[ros_gz_bridge]: Passing message from ROS std_msgs/msg/Float64 to Gazebo gz.msgs.Double
```
If this line appears in the launch output, the gait commands are reaching the joint controllers.

---

## What Each Node Does

| Node | Package | Role |
|------|---------|------|
| `gz sim` | `ros_gz_sim` | Gazebo Harmonic physics + rendering |
| `robot_state_publisher` | `robot_state_publisher` | URDF + `/mcu/joint_states` → `/tf` |
| `ros_gz_sim create` | `ros_gz_sim` | Spawns the hexapod model in the world |
| `parameter_bridge` | `ros_gz_bridge` | Bridges sensor + joint topics between GZ and ROS 2 |
| `gz_odom_bridge.py` | `seeker_gazebo` | Publishes `odom → base_footprint` TF from Gazebo odometry |
| `fake_mcu_node` | `seeker_sim` | Tripod gait simulator; publishes `/mcu/joint_states` + drives GZ joints |

---

## Switching to Hardware

When the physical robot is ready, swap `fake_mcu_node` for the real MCU:

1. Flash `test_bridge_gait` to the ESP32:
   ```bash
   cd ~/mcu_workspaces/seeker_mcu/src/test_bridge_gait
   pio run -e esp32s3sense -t upload
   ```
2. Start the micro-ROS agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```
3. Launch without `fake_mcu_node` (or just stop it):
   ```bash
   ros2 launch seeker_gazebo sim_teleop.launch.py
   # then Ctrl-C fake_mcu_node from the launch output
   ```
   `robot_state_publisher` already remaps `joint_states → /mcu/joint_states`, so
   it will pick up real servo angles from the MCU automatically.

---

## Tuning the Gait

Edit the constants at the top of `seeker_sim/seeker_sim/fake_mcu_node.py`:

| Constant | Default | Effect |
|----------|---------|--------|
| `STEP_HEIGHT` | `0.020` m | How high each foot lifts during swing |
| `CYCLE_TIME` | `1.0` s | Duration of one full tripod cycle |
| `STEP_SCALE` | `1.0` | Step reach multiplier relative to velocity |

Rebuild after changes:
```bash
colcon build --packages-select seeker_sim && source install/setup.bash
```
