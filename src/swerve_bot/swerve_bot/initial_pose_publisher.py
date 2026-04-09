import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos
        )

        self.timer = self.create_timer(3.0, self.timer_callback)
        self.sent = False

    def timer_callback(self):
        if self.sent:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        yaw = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.pub.publish(msg)
        self.get_logger().info('Initial pose published')

        self.sent = True
        self.destroy_timer(self.timer)

def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
