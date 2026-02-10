# Drive Base Control Message
std_msgs/Header header

# Position of the robot
geometry_msgs/TransformStamped transform
geometry_msgs/Twist twist

# Drive Modes
uint8 DRIVE_VECTOR=0  # Command by twist vector
uint8 DRIVE_GOAL=1    # Drive towards goal transform
uint8 drive_mode      # One of DRIVE_VECTOR / DRIVE_GOAL

# Goal position and velocity
# goal_transform is the desired position of the robot in the world frame
# goal_velocity is the desired velocity of the robot in the world frame
# depending on the mode, one of these will be empty.
geometry_msgs/TransformStamped goal_transform
geometry_msgs/Twist goal_velocity