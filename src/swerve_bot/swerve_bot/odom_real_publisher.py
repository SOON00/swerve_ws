import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import struct
import math


class OdomRealPublisher(Node):
    def __init__(self):
        super().__init__('odom_real_publisher')

        # ===== Publishers =====
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== Joints =====
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

        # ===== Serial Connection =====
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.get_logger().info('Serial port opened: /dev/ttyACM0 (115200)')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise e

        # ===== Variables =====
        self.rpm_msg = Float32MultiArray()
        self.rpm_msg.data = [0.0, 0.0, 0.0, 0.0]

        self.angle_msg = Float32MultiArray()
        self.angle_msg.data = [0.0, 0.0, 0.0, 0.0]

        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]
        self.prev_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.lx = [0.17, 0.17, -0.17, -0.17]
        self.ly = [0.15, -0.15, 0.15, -0.15]
        self.vx_modules = [0.0] * 4
        self.vy_modules = [0.0] * 4

        self.wheel_radius = 0.065

        self.STX = bytes([0x53, 0x54, 0x58])
        self.ETX = bytes([0x0D, 0x0A])
        self.packet_size = len(self.STX) + 4*2 + 4*4 + len(self.ETX)  # 29 bytes

        # ===== Timer =====
        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

    def wrap_angle(self, angle):
        return ((angle + math.pi) % (2 * math.pi)) - math.pi

    def read_serial(self):
        if self.serial_port.in_waiting < self.packet_size:
            return False

        buf = self.serial_port.read(self.serial_port.in_waiting)
        stx_idx = buf.find(self.STX)
        if stx_idx == -1 or len(buf) - stx_idx < self.packet_size:
            self.serial_port.reset_input_buffer()
            return False

        data = buf[stx_idx:stx_idx + self.packet_size]
        payload = data[len(self.STX):-len(self.ETX)]

        try:
            RPM_1, RPM_2, RPM_3, RPM_4, ANGLE_1, ANGLE_2, ANGLE_3, ANGLE_4 = struct.unpack('<hhhhffff', payload)
        except struct.error as e:
            self.get_logger().warn(f'Struct unpack error: {e}')
            return False

        # ===== 필터 제거 후 바로 사용 =====
        raw_rpms = [
            RPM_1 + 0.6 if RPM_1 != 0 else 0.0,
            RPM_2 + 0.6 if RPM_2 != 0 else 0.0,
            RPM_3 + 0.6 if RPM_3 != 0 else 0.0,
            RPM_4 + 0.6 if RPM_4 != 0 else 0.0
        ]
        self.rpm_msg.data = raw_rpms
          
        self.angle_msg.data = [
            self.wrap_angle(ANGLE_1),
            self.wrap_angle(ANGLE_2),
            self.wrap_angle(ANGLE_3),
            self.wrap_angle(ANGLE_4)
        ]
        return True

    # ===== JointState + Odom Publish =====
    def loop(self):
        updated = self.read_serial()

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.prev_time = current_time

        wheel_velocities = [rpm * math.pi / 30 for rpm in self.rpm_msg.data]  # rad/s
        for i in range(4):
            self.wheel_pos[i] += wheel_velocities[i] * dt

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.name = self.steer_joints + self.wheel_joints
        joint_state_msg.position = list(self.angle_msg.data) + list(self.wheel_pos)
        joint_state_msg.velocity = [0.0]*4 + wheel_velocities
        joint_state_msg.effort = []
        self.joint_states_pub.publish(joint_state_msg)

        # ===== Odometry 계산 =====
        steer_angles = self.angle_msg.data
        wheel_speeds = wheel_velocities

        for i in range(4):
            v = wheel_speeds[i] * self.wheel_radius
            self.vx_modules[i] = v * math.cos(steer_angles[i])
            self.vy_modules[i] = v * math.sin(steer_angles[i])

        vx = sum(self.vx_modules) / 4.0
        vy = sum(self.vy_modules) / 4.0
        numerator = sum(self.lx[i] * self.vy_modules[i] - self.ly[i] * self.vx_modules[i] for i in range(4))
        denominator = sum(self.lx[i]**2 + self.ly[i]**2 for i in range(4))
        omega = numerator / denominator if denominator != 0 else 0.0

        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRealPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Serial stopped')
    finally:
        if hasattr(node, 'serial_port') and node.serial_port.is_open:
            node.serial_port.close()
            node.get_logger().info('Serial port closed')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
