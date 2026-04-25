import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class StateChecker(Node):
    def __init__(self):
        super().__init__('state_checker')
        self.trigger_sub = self.create_subscription(Bool, '/search_trigger', self.trigger_cb, 10)
        self.target_sub = self.create_subscription(String, '/target_object', self.target_cb, 10)
        self.trigger_received = False
        self.target_received = False

    def trigger_cb(self, msg):
        self.get_logger().info(f'SEARCH TRIGGER: {msg.data}')
        self.trigger_received = True

    def target_cb(self, msg):
        self.get_logger().info(f'TARGET OBJECT: {msg.data}')
        self.target_received = True

def main():
    rclpy.init()
    node = StateChecker()
    print("Checking for triggers... (Timeout in 5s)")
    
    # Spin for a bit to catch current state if latched, or next published
    import time
    start = time.time()
    while rclpy.ok() and time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.trigger_received and node.target_received:
            break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
