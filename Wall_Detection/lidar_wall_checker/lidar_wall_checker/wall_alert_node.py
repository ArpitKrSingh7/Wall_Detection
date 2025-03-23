import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String  # You can change String to Int32 etc.

class WallAlertNode(Node):
    def __init__(self):
        super().__init__('wall_alert_node')
        self.subscription = self.create_subscription(
            Bool,
            '/robot_near_wall',
            self.near_wall_callback,
            10
        )
        self.alert_publisher = self.create_publisher(String, '/wall_alert', 10)
        self.get_logger().info("Wall Alert Node Started")

    def near_wall_callback(self, msg):
        alert_msg = String()
        if msg.data:  # If near wall
            alert_msg.data = "⚠️ Warning: Robot is near the wall!"
        else:
            alert_msg.data = "✅ Robot is clear."
        self.alert_publisher.publish(alert_msg)
        self.get_logger().info(f"Published Alert: {alert_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = WallAlertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
