# ros2_ws

ROS 2 Jazzy colcon workspace. Packages live in `src/`:

| Package | Purpose |
|---|---|
| `mcu_msgs` | Shared ROS 2 message/service definitions (`.msg`/`.srv`) used by both ROS 2 nodes and micro-ROS MCU firmware. Also mounted into `mcu_ws/extra_packages/` by Docker. |
| `test_package` | Minimal C++ ROS 2 node for workflow verification. |

## Build

Inside the Docker dev container:

```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```
