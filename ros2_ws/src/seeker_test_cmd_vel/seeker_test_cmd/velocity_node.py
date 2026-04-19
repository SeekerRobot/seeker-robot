import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VelocityNode(Node):
    def __init__(self):
        super().__init__("velocity_node")

        # Declare parameters that can be changed by launch files
        self.declare_parameter("drive_mode", "manual")
        self.declare_parameter("linear_speed", 0.5)

        # Subscribers and Publishers
        self.subscription = self.create_subscription(String, "/voice_command", self.command_callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Safety timer
        self.stop_timer = None
        self.move_duration = 2.0

        mode = self.get_parameter("drive_mode").get_parameter_value().string_value
        speed = self.get_parameter("linear_speed").get_parameter_value().double_value
        self.get_logger().info(f"Velocity node started in {mode} mode. Base speed: {speed} m/s. Listening to /voice_command...")

    def command_callback(self, msg: String):
        command = msg.data
        
        # Fetch the speed dynamically in case it changed
        base_speed = self.get_parameter("linear_speed").get_parameter_value().double_value

        self.get_logger().info(f"Received command: '{command}'. Translating to velocity...")
        twist_msg = Twist()

        if command == "move forward":
            twist_msg.linear.x = base_speed
        elif command == "move backward":
            twist_msg.linear.x = -base_speed
        elif command == "move left":
            twist_msg.angular.z = 1.0
        elif command == "move right":
            twist_msg.angular.z = -1.0
        elif command == "spin":
            twist_msg.angular.z = 3.0
        elif command == "dance":
            twist_msg.linear.x = base_speed / 2.0
            twist_msg.angular.z = 2.0
        elif command == "find the object":
            twist_msg.angular.z = 0.5
        elif command == "NONE OF THE ABOVE":
            self.get_logger().warn("Command not recognized. Stopping.")
        else:
            self.get_logger().error(f"Unknown command received: {command}")
            return

        self.publisher.publish(twist_msg)
        self.start_stop_timer()

    def start_stop_timer(self):
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(self.move_duration, self.stop_robot)

    def stop_robot(self):
        self.publisher.publish(Twist())
        self.get_logger().info("Movement complete. Stopping robot.")
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