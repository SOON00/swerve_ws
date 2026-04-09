import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class SwerveOdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_imu_publisher')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        # Wheel Offset
        self.lx = [1.3, 1.3, -1.3, -1.3]
        self.ly = [0.1, -0.1, 0.1, -0.1]

        self.vx_modules = [0.0] * 4
        self.vy_modules = [0.0] * 4

        self.steer_joints = [
            "front_left_steer_joint",
            "front_right_steer_joint",
            "rear_left_steer_joint",
            "rear_right_steer_joint"
        ]
        self.wheel_joints = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint"
        ]
        
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def joint_state_callback(self, msg: JointState):
        current_time = msg.header.stamp
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (rclpy.time.Time.from_msg(current_time) -
              rclpy.time.Time.from_msg(self.last_time)).nanoseconds / 1e9
        self.last_time = current_time

        steer_angles = [0.0] * 4
        wheel_speeds = [0.0] * 4

        for i, name in enumerate(msg.name):
            if name in self.steer_joints:
                idx = self.steer_joints.index(name)
                steer_angles[idx] = msg.position[i]  # rad
            elif name in self.wheel_joints:
                idx = self.wheel_joints.index(name)
                wheel_speeds[idx] = msg.velocity[i]  # rad/s

        wheel_radius = 0.065

        for i in range(4):
            v = wheel_speeds[i] * wheel_radius
            self.vx_modules[i] = v * math.cos(steer_angles[i])
            self.vy_modules[i] = v * math.sin(steer_angles[i])

        vx = sum(self.vx_modules) / 4.0
        vy = sum(self.vy_modules) / 4.0
        numerator = sum(self.lx[i] * self.vy_modules[i] - self.ly[i] * self.vx_modules[i] for i in range(4))
        denominator = sum(self.lx[i]**2 + self.ly[i]**2 for i in range(4))
        omega = numerator / denominator if denominator != 0 else 0.0

        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.x += delta_x
        self.y += delta_y
        
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()