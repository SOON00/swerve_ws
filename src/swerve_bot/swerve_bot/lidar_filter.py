#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class BackLaserFilter(Node):
    def __init__(self):
        super().__init__("laser_filter")
        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(
            LaserScan,
            "/scan_filtered",
            10
        )
        self.get_logger().info("Back 180° laser filter started.")

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered = msg  # shallow copy OK
        filtered.ranges = list(msg.ranges)  # deep copy

        angle = msg.angle_min
        for i in range(len(msg.ranges)):
            deg = math.degrees(angle)

            # 뒤쪽 180° 제거 → |deg| > 90°
            if abs(deg) < 90:
                filtered.ranges[i] = float('inf')

            angle += msg.angle_increment

        self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = BackLaserFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
