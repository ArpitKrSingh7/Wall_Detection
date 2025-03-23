import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class BeepAlertNode(Node):
    def __init__(self):
        super().__init__('beep_alert_node')
        self.subscription = self.create_subscription(
            String,
            '/wall_alert',
            self.alert_callback,
            10
        )
        self.get_logger().info("Beep Alert Node Started")

    def alert_callback(self, msg):
        if "Warning" in msg.data:
            # System beep (works in many terminals)
            os.system('echo -e "\a"')
            self.get_logger().info("🚨 Beep! Robot near wall!")
        else:
            # No beep needed, optional log
            pass

def main(args=None):
    rclpy.init(args=args)
    node = BeepAlertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
