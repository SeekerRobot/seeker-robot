# /mcu/hexapod_cmd — gait mode and body pose control
# The MCU subscribes to this topic. Published by the ROS 2 mission planner
# or teleop tools to switch gait states and adjust body orientation.

uint8 MODE_STAND = 0
uint8 MODE_WALK  = 1
uint8 MODE_SIT   = 2
uint8 MODE_DANCE = 3

uint8   mode          # gait mode (see constants above)
float32 body_height   # body height offset from neutral (mm, positive = higher)
float32 body_pitch    # body pitch (degrees, forward-down positive)
float32 body_roll     # body roll (degrees, right-down positive)