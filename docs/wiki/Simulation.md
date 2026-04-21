# Simulation

Every simulation mode is self-contained: one `ros2 launch` command brings up Gazebo Harmonic, the hexapod model, the fake MCU gait, and whatever autonomy stack you asked for. The same Python tripod gait used in simulation (`seeker_sim/fake_mcu_node`) replaces the ESP32, so there's no firmware or hardware needed to drive the robot.

All commands below assume you are inside the dev container:

```bash
docker compose exec ros2 bash
# then for each new shell:
source /opt/ros/jazzy/setup.bash
source ~/ros2_workspaces/install/setup.bash
```

You must have built at least:

```bash
colcon build --packages-select mcu_msgs seeker_description seeker_display seeker_sim seeker_gazebo seeker_media seeker_navigation seeker_vision
```

---

## `sim_teleop` — manual drive (start here)

Brings up Gazebo, spawns the hexapod in `slam_test.sdf`, runs `robot_state_publisher`, the ROS↔Gazebo bridge, the ground-truth `odom → base_footprint` TF bridge, and `fake_mcu_node` for walking. RViz opens with `teleop.rviz`.

```bash
# Terminal 1
ros2 launch seeker_gazebo sim_teleop.launch.py

# Terminal 2 (drive the robot)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Teleop keys:**

| Key | Action |
|---|---|
| `i` | Forward |
| `,` | Backward |
| `j` / `l` | Rotate left / right |
| `k` | Stop cleanly |
| `q` / `z` | Increase / decrease linear speed |
| `w` / `x` | Increase / decrease angular speed |

**What you should see:**

1. Gazebo opens with the walled test world and the hexapod spawned.
2. Pressing `i` makes the legs walk in alternating tripod (FL+MR+RL vs FR+ML+RR).
3. RViz shows the TF tree with all 12 articulated joints.

**Verify topics** (run in the *same* shell that launched the sim — Docker DDS multicast can be flaky across `docker exec` shells):

```bash
ros2 topic hz /mcu/joint_states   # ~100 Hz  (from fake_mcu_node)
ros2 topic hz /mcu/imu            # ~200 Hz  (bridged from Gazebo IMU plugin via ros_gz_bridge)
ros2 topic hz /mcu/scan           # ~10 Hz   (bridged from Gazebo laser plugin)
ros2 topic echo /cmd_vel          # prints Twist while teleop key is held
ros2 run tf2_tools view_frames    # emit a PDF of the full TF tree
```

Only `/mcu/joint_states` comes from `fake_mcu_node` directly; `/mcu/imu`, `/mcu/scan`, `/odom`, and `/camera/image` are bridged from Gazebo plugins via `ros_gz_bridge` using `seeker_gazebo/config/bridge.yaml`.

**RViz fixed frame:** `odom`

---

## `sim_slam_raw` — SLAM with ground-truth odometry

Same base as `sim_teleop`, plus `slam_toolbox` running in async online mode. Odometry still comes from Gazebo ground truth via `gz_odom_bridge.py`, so there's **no EKF and no IMU fusion** — the hexapod walks but the TF chain is effectively flat. This is the simplest way to verify SLAM works without touching the real-hardware pipeline.

```bash
ros2 launch seeker_gazebo sim_slam_raw.launch.py
# drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Verify:**

```bash
ros2 lifecycle get /slam_toolbox  # expect "active [3]"
ros2 topic hz /map                # ~0.2 Hz
ros2 topic echo /map --once | head
```

**RViz fixed frame:** `odom` (add `Map` with `Topic Durability: Transient Local`, `LaserScan`, `TF`, `RobotModel`)

---

## `sim_slam_ekf` — SLAM with IMU-fused EKF odometry

Same as `sim_slam_raw`, but `fake_mcu_node`'s `/mcu/imu` topic is fused into the `odom → base_footprint` TF by `robot_localization`'s EKF. This exactly mirrors the real-hardware pipeline, so it's the best sim mode for regression testing before deploying.

```bash
ros2 launch seeker_gazebo sim_slam_ekf.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Verify:**

```bash
ros2 topic hz /odometry/filtered    # EKF output
ros2 run tf2_ros tf2_echo odom base_footprint
# Tilting the robot in Gazebo (drive it up a ramp) should show roll/pitch changing
```

**RViz fixed frame:** `odom`

---

## `sim_object_seek` — YOLO-driven full autonomy demo

Full stack: Gazebo + fake MCU + EKF + SLAM Toolbox + Nav2 + `gazebo_vision_node` (YOLOv8 on all 80 COCO classes) + `object_seeker`. The robot starts in **WANDER** mode and autonomously frontier-explores the world. Send it a `SeekObject` action goal at any time and it will reset its map, re-explore, and approach the target.

The world (`slam_test.sdf`) contains three COCO-class props from Gazebo Fuel: a **sports ball** (NW corner), a **teddy bear** (NE corner), and a **chair** (SW corner).

```bash
ros2 launch seeker_gazebo sim_object_seek.launch.py
```

The launch is gated behind timers to avoid lifecycle race conditions:

```
t=0s   Gazebo + fake_mcu + robot_state_publisher + gz_bridge
t=2s   EKF + scan_tilt_filter
t=5s   SLAM Toolbox + gazebo_vision_node (YOLOv8)
t=10s  slam_toolbox configure → activate
t=13s  Nav2 nodes
t=15s  Nav2 lifecycle manager
t=25s  object_seeker (starts in WANDER / frontier exploration)
```

**Send a seek goal (friendly CLI):**

```bash
# Underscores are converted to spaces automatically
ros2 run seeker_navigation find teddy_bear --feedback
ros2 run seeker_navigation find sports_ball --timeout 120 -f
ros2 run seeker_navigation find chair

# Raw action equivalent:
ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \
  "{class_name: 'sports ball', timeout_sec: 60.0}" --feedback
```

Exit codes: `0` = found, `1` = rejected/server unavailable, `2` = timeout/failed, `130` = cancelled (Ctrl-C).

Each new seek goal **resets the SLAM map and Nav2 costmaps** so the robot re-explores from scratch rather than replaying a stale map.

**Other controls:**

```bash
# Return to idle exploration at any time:
ros2 service call /wander std_srvs/srv/Trigger

# Trigger a dance:
ros2 service call /perform_move mcu_msgs/srv/PerformMove \
  "{move_name: 'dance', duration_sec: 3.0}"
```

**Verify the stack is healthy:**

```bash
ros2 lifecycle get /slam_toolbox          # active [3]
ros2 lifecycle get /controller_server     # active [3]
ros2 lifecycle get /planner_server        # active [3]
ros2 topic echo /vision/detections --once # YOLO hits when object is in view
ros2 topic echo /navigate_to_pose/_action/status
ros2 topic echo /cmd_vel
```

**RViz fixed frame:** `map`
Recommended displays: `Map` (`/map`, Transient Local), `LaserScan` (`/mcu/scan`), `RobotModel`, `TF`, `Path` (`/plan`), `Costmap (local)` (`/local_costmap/costmap`), `Costmap (global)` (`/global_costmap/costmap`).

---

## Adding standalone YOLO vision to any simulation run

`gazebo_vision_node` can also be run alongside any launch that publishes `/camera/image`:

```bash
# Terminal 1
ros2 launch seeker_gazebo sim_teleop.launch.py

# Terminal 2 — publishes all YOLO detections on /vision/detections
ros2 run seeker_vision gazebo_vision_node
```

`/vision/detections` (`mcu_msgs/DetectedObjectArray`) carries every COCO class seen in the frame — class name, confidence, bounding box centre/size, and image dimensions for bearing computation.

---

## Display-only (no physics)

If you just want to see the URDF in RViz without running Gazebo:

```bash
ros2 launch seeker_description display.launch.py
```

Fixed frame: `base_link`. Useful for URDF debugging.

---

## Tuning the gait

Edit the constants at the top of `ros2_ws/src/seeker_sim/seeker_sim/fake_mcu_node.py`:

| Constant | Default | Effect |
|---|---|---|
| `CYCLE_TIME` | 0.8 s | Full tripod cycle duration |
| `STRIDE_HIP` | 45° | Maximum hip sweep amplitude per side |
| `NEUTRAL_KNEE` | 30° | Standing knee angle (sets body height) |
| `LIFT_KNEE` | 88° | Peak knee angle during swing (foot clearance) |
| `VX_MAX` | 0.4 m/s | Commanded speed that maps to full `STRIDE_HIP` |
| `WZ_MAX` | 1.2 rad/s | Yaw rate that maps to half `STRIDE_HIP` of turn amplitude |

Body height is also adjustable at runtime via `linear.z` on `/cmd_vel` (the `t` / `b` keys in `teleop_twist_keyboard`), which walks `NEUTRAL_KNEE` between `NEUTRAL_KNEE_MIN` (5°) and `NEUTRAL_KNEE_MAX` (`LIFT_KNEE` − 10°).

Rebuild:

```bash
colcon build --packages-select seeker_sim --symlink-install
source install/setup.bash
```

(with `--symlink-install`, pure Python edits don't even need a rebuild — just restart the launch).

---

## Debugging tips

| Problem | What to try |
|---|---|
| `ros2 topic hz` reports *No new messages* but Gazebo is clearly simulating | Known DDS multicast isolation quirk — run the command in the *same* shell that launched the sim, not a fresh `docker exec`. |
| Legs glitch / robot explodes on spawn | The `gz_spawn` node sets a `z=0.10` clearance. If you changed `NEUTRAL_KNEE` in `fake_mcu_node.py`, you may need to raise the spawn height so the body doesn't clip the ground. |
| SLAM never activates in `sim_slam_ekf` | Check `ros2 lifecycle get /slam_toolbox`. If it's stuck in `unconfigured`, the `slam_activate` `ExecuteProcess` likely errored — look for `configure` failures in the launch output, often due to a missing TF (EKF not yet publishing). Wait longer or restart the launch. |
| `object_seeker` sits idle after t=25 s | Check Nav2 goal acceptance: `ros2 topic echo /navigate_to_pose/_action/status`. If you see `STATUS_REJECTED`, SLAM or the costmaps aren't fully alive yet. |
| YOLO never fires on the target | Run `ros2 topic echo /vision/detections --once` while the target is in camera view. If empty, check `gazebo_vision_node` is running (`ros2 node list`). Models are downloaded from Gazebo Fuel on first launch — ensure internet access or pre-cache them in `~/.gz/fuel`. |
| Gazebo window shows the robot but no sensors publish | Check `ros2 node list` — if `ros_gz_bridge` is missing, `bridge.yaml` failed to load. The launch log will say which topic it choked on. |
| You only see `base_link` in RViz, no map or laser | Your RViz fixed frame is wrong for this launch mode. Match the table above. |

---

## Switching from sim to hardware

When you're ready to replace `fake_mcu_node` with a real ESP32:

1. Flash `test_bridge_all` (all publishers) or `test_bridge_gait` (just gait + `/cmd_vel`).
2. Start the micro-ROS agent in a new terminal: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`.
3. Launch one of the `real_*` launches in `seeker_navigation` (see **[ROS2 Packages → seeker_navigation](ROS2-Packages.md#seeker_navigation)** and **[IRL Tests](IRL-Tests.md)**).

`robot_state_publisher` in every simulation launch already remaps `joint_states → /mcu/joint_states`, so it picks up servo angles from the real MCU micro-ROS bridge automatically.
