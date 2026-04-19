import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VelocityNode(Node):
    def __init__(self):
        super().__init__("velocity_node")

        # 1. Subscribe to the voice commands from the Gemini node
        self.subscription = self.create_subscription(
            String,
            "/voice_command",
            self.command_callback,
            10
        )

        # 2. Publish to the standard ROS 2 velocity topic
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # 3. Setup a safety timer variable to stop the robot
        self.stop_timer = None
        self.move_duration = 2.0  # How long the robot moves per command

        self.get_logger().info("Velocity node started. Listening to /voice_command...")

    def command_callback(self, msg: String):
        command = msg.data
        self.get_logger().info(f"Received command: '{command}'. Translating to velocity...")

        # Create an empty Twist message (defaults all velocities to 0.0)
        twist_msg = Twist()

        # Map the string command to specific speeds
        if command == "move forward":
            twist_msg.linear.x = 0.5    # Move forward at 0.5 m/s
        elif command == "move backward":
            twist_msg.linear.x = -0.5   # Move backward
        elif command == "move left":
            twist_msg.angular.z = 1.0   # Turn left (counter-clockwise) at 1 rad/s
        elif command == "move right":
            twist_msg.angular.z = -1.0  # Turn right (clockwise)
        elif command == "spin":
            twist_msg.angular.z = 3.0   # Spin fast!
        elif command == "dance":
            # For a simple test, we'll just do a wobbly forward motion
            twist_msg.linear.x = 0.3
            twist_msg.angular.z = 2.0
        elif command == "find the object":
            # Spin slowly to "look" around
            twist_msg.angular.z = 0.5
        elif command == "NONE OF THE ABOVE":
            self.get_logger().warn("Command not recognized by AI. Stopping.")
            # Everything is 0.0 by default, so it just stops
        else:
            self.get_logger().error(f"Unknown command received: {command}")
            return

        # Publish the movement
        self.publisher.publish(twist_msg)

        # Start a timer to stop the robot after `move_duration` seconds
        self.start_stop_timer()

    def start_stop_timer(self):
        # Cancel any existing timer so commands can chain smoothly
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        
        # Create a new one-shot timer that calls the stop_robot function
        self.stop_timer = self.create_timer(self.move_duration, self.stop_robot)

    def stop_robot(self):
        # Send a zero-velocity message
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        self.get_logger().info("Movement complete. Stopping robot.")
        
        # Cancel the timer so it doesn't loop
        self.stop_timer.cancel()
        self.stop_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()