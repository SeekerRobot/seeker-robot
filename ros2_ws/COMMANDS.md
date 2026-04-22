# Seeker Robot — Simulation & Integration Commands

This guide provides the exact commands used to launch, monitor, and control the autonomous seeker robot in Gazebo.

## 1. Startup
Launch the full autonomy stack (Gazebo + Nav2 + SLAM + YOLO + Object Seeker).
```bash
docker exec -it -e DISPLAY=:0 seeker-robot-ros2-1 bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_gazebo sim_integrated_medium.launch.py"
```

## 2. Triggering a Search
Send an action goal to the robot to find and approach a specific object.

**Find Soccer Ball:**
```bash
docker exec -it seeker-robot-ros2-1 bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \"{class_name: 'sports ball', timeout_sec: 300.0}\" --feedback"
```

**Find Teddy Bear:**
```bash
docker exec -it seeker-robot-ros2-1 bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \"{class_name: 'teddy bear', timeout_sec: 300.0}\" --feedback"
```

## 3. Monitoring & Debugging
Check the robot's internal state machine and search progress.

**Live Feedback (State & Area):**
```bash
docker exec seeker-robot-ros2-1 bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 topic echo /seek_object/_action/feedback"
```

**Follow Logs:**
```bash
docker exec -it seeker-robot-ros2-1 bash -c "tail -f /home/ubuntu/.ros/log/latest/launch.log"
```

---
*Note: Ensure the real YOLO weights are downloaded to `install/seeker_vision/share/seeker_vision/model/yolo26n.pt` for object detection to work.*
