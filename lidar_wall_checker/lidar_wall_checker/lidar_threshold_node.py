import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class LidarThresholdNode(Node):
    def __init__(self):
        super().__init__('lidar_threshold_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(Bool, '/robot_near_wall', 10)
        self.threshold = 1.0  # Threshold distance in meters

    def lidar_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not math.isinf(r)]
        if valid_ranges:
            min_distance = min(valid_ranges)
            is_near = min_distance < self.threshold
        else:
            is_near = False
        
        bool_msg = Bool()
        bool_msg.data = is_near
        self.publisher.publish(bool_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarThresholdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
