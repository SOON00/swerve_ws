# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, Float64
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import serial
# import struct
# import math


# class OdomRealPublisher(Node):
#     def __init__(self):
#         super().__init__('odom_real_imu_publisher')

#         # ===== Publishers =====
#         self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         # ==== Subscribers ====
#         self.imu_sub = self.create_subscription(Float64, '/imu/yaw', self.imu_callback, 10)

#         # ===== Joints =====
#         self.steer_joints = [
#             "front_left_steer_joint",
#             "front_right_steer_joint",
#             "rear_left_steer_joint",
#             "rear_right_steer_joint"
#         ]
#         self.wheel_joints = [
#             "front_left_wheel_joint",
#             "front_right_wheel_joint",
#             "rear_left_wheel_joint",
#             "rear_right_wheel_joint"
#         ]

#         # ===== Serial Connection =====
#         try:
#             self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
#             self.serial_port.reset_input_buffer()
#             self.serial_port.reset_output_buffer()
#             self.get_logger().info('Serial port opened: /dev/ttyACM0 (115200)')
#         except serial.SerialException as e:
#             self.get_logger().error(f'Failed to open serial port: {e}')
#             raise e

#         # ===== Variables =====
#         self.rpm_msg = Float32MultiArray()
#         self.rpm_msg.data = [0.0, 0.0, 0.0, 0.0]

#         self.angle_msg = Float32MultiArray()
#         self.angle_msg.data = [0.0, 0.0, 0.0, 0.0]

#         self.wheel_pos = [0.0, 0.0, 0.0, 0.0]
#         self.prev_time = self.get_clock().now()

#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.theta_imu = 0.0
#         self.alpha = 0.9   # Complementary Filter Alpha-Gain
        
#         self.prev_theta_imu_raw = None
#         self.theta_imu_unwrapped = 0.0

#         self.lx = [0.35, 0.35, -0.35, -0.35]
#         self.ly = [0.15, -0.15, 0.15, -0.15]
#         self.vx_modules = [0.0] * 4
#         self.vy_modules = [0.0] * 4

#         self.wheel_radius = 0.065

#         self.STX = bytes([0x53, 0x54, 0x58])
#         self.ETX = bytes([0x0D, 0x0A])
#         self.packet_size = len(self.STX) + 4*2 + 4*4 + len(self.ETX)  # 29 bytes

#         # ===== Timer =====
#         self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

#     def wrap_angle(self, angle):
#         return ((angle + math.pi) % (2 * math.pi)) - math.pi

#     def imu_callback(self, msg):
#         theta_raw = msg.data * math.pi / 180.0  # rad 단위

#         if self.prev_theta_imu_raw is not None:
#             diff = theta_raw - self.prev_theta_imu_raw
#             # wrap-around 보정
#             if diff > math.pi:
#                 diff -= 2 * math.pi
#             elif diff < -math.pi:
#                 diff += 2 * math.pi
#             self.theta_imu_unwrapped += diff

#         self.prev_theta_imu_raw = theta_raw
#         self.theta_imu = self.theta_imu_unwrapped

#     def read_serial(self):
#         if self.serial_port.in_waiting < self.packet_size:
#             return False

#         buf = self.serial_port.read(self.serial_port.in_waiting)
#         stx_idx = buf.find(self.STX)
#         if stx_idx == -1 or len(buf) - stx_idx < self.packet_size:
#             self.serial_port.reset_input_buffer()
#             return False

#         data = buf[stx_idx:stx_idx + self.packet_size]
#         payload = data[len(self.STX):-len(self.ETX)]

#         try:
#             RPM_1, RPM_2, RPM_3, RPM_4, ANGLE_1, ANGLE_2, ANGLE_3, ANGLE_4 = struct.unpack('<hhhhffff', payload)
#         except struct.error as e:
#             self.get_logger().warn(f'Struct unpack error: {e}')
#             return False

#         # ===== 필터 제거 후 바로 사용 =====
#         raw_rpms = [
#             RPM_1 * 0.1 if RPM_1 != 0 else 0.0,
#             RPM_2 * 0.1 if RPM_2 != 0 else 0.0,
#             RPM_3 * 0.1 if RPM_3 != 0 else 0.0,
#             RPM_4 * 0.1 if RPM_4 != 0 else 0.0
#         ]
#         self.rpm_msg.data = raw_rpms
          
#         self.angle_msg.data = [
#             self.wrap_angle(ANGLE_1),
#             self.wrap_angle(ANGLE_2),
#             self.wrap_angle(ANGLE_3),
#             self.wrap_angle(ANGLE_4)
#         ]
#         return True

#     # ===== JointState + Odom Publish =====
#     def loop(self):
#         updated = self.read_serial()

#         current_time = self.get_clock().now()
#         dt = (current_time - self.prev_time).nanoseconds / 1e9
#         if dt <= 0:
#             return
#         self.prev_time = current_time

#         wheel_velocities = [rpm * math.pi / 30 for rpm in self.rpm_msg.data]  # rad/s
#         for i in range(4):
#             self.wheel_pos[i] += wheel_velocities[i] * dt

#         joint_state_msg = JointState()
#         joint_state_msg.header.stamp = current_time.to_msg()
#         joint_state_msg.name = self.steer_joints + self.wheel_joints
#         joint_state_msg.position = list(self.angle_msg.data) + list(self.wheel_pos)
#         joint_state_msg.velocity = [0.0]*4 + wheel_velocities
#         joint_state_msg.effort = []
#         self.joint_states_pub.publish(joint_state_msg)

#         # ===== Odometry 계산 =====
#         steer_angles = self.angle_msg.data
#         wheel_speeds = wheel_velocities

#         for i in range(4):
#             v = wheel_speeds[i] * self.wheel_radius
#             self.vx_modules[i] = v * math.cos(steer_angles[i])
#             self.vy_modules[i] = v * math.sin(steer_angles[i])

#         vx = sum(self.vx_modules) / 4.0
#         vy = sum(self.vy_modules) / 4.0
#         numerator = sum(self.lx[i] * self.vy_modules[i] - self.ly[i] * self.vx_modules[i] for i in range(4))
#         denominator = sum(self.lx[i]**2 + self.ly[i]**2 for i in range(4))
#         omega = numerator / denominator if denominator != 0 else 0.0

#         delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
#         delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
#         theta_odom = omega * dt

#         self.x += delta_x
#         self.y += delta_y
#         self.theta = self.alpha * (self.theta + theta_odom) + (1 - self.alpha) * self.theta_imu 

#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_footprint'
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
#         odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
#         odom_msg.twist.twist.linear.x = vx
#         odom_msg.twist.twist.linear.y = vy
#         odom_msg.twist.twist.angular.z = omega

#         self.odom_pub.publish(odom_msg)

#         t = TransformStamped()
#         t.header.stamp = current_time.to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_footprint'
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.rotation.z = math.sin(self.theta / 2.0)
#         t.transform.rotation.w = math.cos(self.theta / 2.0)
#         self.tf_broadcaster.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomRealPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Serial stopped')
#     finally:
#         if hasattr(node, 'serial_port') and node.serial_port.is_open:
#             node.serial_port.close()
#             node.get_logger().info('Serial port closed')
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()





# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, Float64
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped, Twist
# from tf2_ros import TransformBroadcaster
# import serial
# import struct
# import math

# class OdomRealPublisher(Node):
#     def __init__(self):
#         super().__init__('odom_real_imu_publisher')

#         # ===== Publishers =====
#         self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         # ==== Subscribers ====
#         self.imu_sub = self.create_subscription(Float64, '/imu/yaw', self.imu_callback, 10)
#         self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

#         # ===== Joints =====
#         self.steer_joints = [
#             "front_left_steer_joint",
#             "front_right_steer_joint",
#             "rear_left_steer_joint",
#             "rear_right_steer_joint"
#         ]
#         self.wheel_joints = [
#             "front_left_wheel_joint",
#             "front_right_wheel_joint",
#             "rear_left_wheel_joint",
#             "rear_right_wheel_joint"
#         ]

#         # ===== Serial Connection =====
#         try:
#             self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
#             self.serial_port.reset_input_buffer()
#             self.serial_port.reset_output_buffer()
#             self.get_logger().info('Serial port opened: /dev/ttyACM0 (115200)')
#         except serial.SerialException as e:
#             self.get_logger().error(f'Failed to open serial port: {e}')
#             raise e

#         # ===== Variables =====
#         self.rpm_msg = Float32MultiArray()
#         self.rpm_msg.data = [0.0, 0.0, 0.0, 0.0]

#         self.angle_msg = Float32MultiArray()
#         self.angle_msg.data = [0.0, 0.0, 0.0, 0.0]

#         self.wheel_pos = [0.0, 0.0, 0.0, 0.0]
#         self.prev_time = self.get_clock().now()

#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.theta_imu = 0.0
#         self.alpha = 0.9   # Complementary Filter Alpha-Gain
        
#         self.prev_theta_imu_raw = None
#         self.theta_imu_unwrapped = 0.0

#         self.lx = [0.35, 0.35, -0.35, -0.35]
#         self.ly = [0.15, -0.15, 0.15, -0.15]
#         self.vx_modules = [0.0] * 4
#         self.vy_modules = [0.0] * 4

#         self.wheel_radius = 0.065

#         self.STX = bytes([0x53, 0x54, 0x58])
#         self.ETX = bytes([0x0D, 0x0A])
#         self.packet_size = len(self.STX) + 4*2 + 4*4 + len(self.ETX)  # 29 bytes

#         # ===== cmd_vel =====
#         self.cmd_vel = Twist()
#         self.cmd_received = False

#         # ===== Timer =====
#         self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

#     def wrap_angle(self, angle):
#         return ((angle + math.pi) % (2 * math.pi)) - math.pi

#     def imu_callback(self, msg):
#         theta_raw = msg.data * math.pi / 180.0  # rad

#         if self.prev_theta_imu_raw is not None:
#             diff = theta_raw - self.prev_theta_imu_raw
#             if diff > math.pi:
#                 diff -= 2 * math.pi
#             elif diff < -math.pi:
#                 diff += 2 * math.pi
#             self.theta_imu_unwrapped += diff

#         self.prev_theta_imu_raw = theta_raw
#         self.theta_imu = self.theta_imu_unwrapped

#     def cmd_callback(self, msg: Twist):
#         self.cmd_vel = msg
#         self.cmd_received = True

#     def read_serial(self):
#         if self.serial_port.in_waiting < self.packet_size:
#             return False

#         buf = self.serial_port.read(self.serial_port.in_waiting)
#         stx_idx = buf.find(self.STX)
#         if stx_idx == -1 or len(buf) - stx_idx < self.packet_size:
#             self.serial_port.reset_input_buffer()
#             return False

#         data = buf[stx_idx:stx_idx + self.packet_size]
#         payload = data[len(self.STX):-len(self.ETX)]

#         try:
#             RPM_1, RPM_2, RPM_3, RPM_4, ANGLE_1, ANGLE_2, ANGLE_3, ANGLE_4 = struct.unpack('<hhhhffff', payload)
#         except struct.error as e:
#             self.get_logger().warn(f'Struct unpack error: {e}')
#             return False

#         raw_rpms = [
#             RPM_1 * 0.1 if RPM_1 != 0 else 0.0,
#             RPM_2 * 0.1 if RPM_2 != 0 else 0.0,
#             RPM_3 * 0.1 if RPM_3 != 0 else 0.0,
#             RPM_4 * 0.1 if RPM_4 != 0 else 0.0
#         ]
#         self.rpm_msg.data = raw_rpms
          
#         self.angle_msg.data = [
#             self.wrap_angle(ANGLE_1),
#             self.wrap_angle(ANGLE_2),
#             self.wrap_angle(ANGLE_3),
#             self.wrap_angle(ANGLE_4)
#         ]
#         return True

#     def loop(self):
#         updated = self.read_serial()

#         current_time = self.get_clock().now()
#         dt = (current_time - self.prev_time).nanoseconds / 1e9
#         if dt <= 0:
#             return
#         self.prev_time = current_time

#         # ===== Wheel velocities =====
#         wheel_velocities = [rpm * math.pi / 30 for rpm in self.rpm_msg.data]  # rad/s
#         for i in range(4):
#             self.wheel_pos[i] += wheel_velocities[i] * dt

#         joint_state_msg = JointState()
#         joint_state_msg.header.stamp = current_time.to_msg()
#         joint_state_msg.name = self.steer_joints + self.wheel_joints
#         joint_state_msg.position = list(self.angle_msg.data) + list(self.wheel_pos)
#         joint_state_msg.velocity = [0.0]*4 + wheel_velocities
#         joint_state_msg.effort = []
#         self.joint_states_pub.publish(joint_state_msg)

#         # ===== Odometry 계산 =====
#         if self.cmd_received:
#             vx = self.cmd_vel.linear.x
#             vy = self.cmd_vel.linear.y
#             omega = self.cmd_vel.angular.z
#         else:
#             steer_angles = self.angle_msg.data
#             wheel_speeds = wheel_velocities
#             for i in range(4):
#                 v = wheel_speeds[i] * self.wheel_radius
#                 self.vx_modules[i] = v * math.cos(steer_angles[i])
#                 self.vy_modules[i] = v * math.sin(steer_angles[i])

#             vx = sum(self.vx_modules) / 4.0
#             vy = sum(self.vy_modules) / 4.0
#             numerator = sum(self.lx[i] * self.vy_modules[i] - self.ly[i] * self.vx_modules[i] for i in range(4))
#             denominator = sum(self.lx[i]**2 + self.ly[i]**2 for i in range(4))
#             omega = numerator / denominator if denominator != 0 else 0.0

#         delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
#         delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
#         theta_odom = omega * dt

#         self.x += delta_x
#         self.y += delta_y
#         self.theta = self.alpha * (self.theta + theta_odom) + (1 - self.alpha) * self.theta_imu 

#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_footprint'
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
#         odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
#         odom_msg.twist.twist.linear.x = vx
#         odom_msg.twist.twist.linear.y = vy
#         odom_msg.twist.twist.angular.z = omega

#         self.odom_pub.publish(odom_msg)

#         t = TransformStamped()
#         t.header.stamp = current_time.to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_footprint'
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.rotation.z = math.sin(self.theta / 2.0)
#         t.transform.rotation.w = math.cos(self.theta / 2.0)
#         self.tf_broadcaster.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomRealPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Serial stopped')
#     finally:
#         if hasattr(node, 'serial_port') and node.serial_port.is_open:
#             node.serial_port.close()
#             node.get_logger().info('Serial port closed')
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped, Twist
# from tf2_ros import TransformBroadcaster

# import serial
# import struct
# import math


# class OdomCmdvelSerialNode(Node):
#     def __init__(self):
#         super().__init__('odom_cmdvel_serial_node')

#         # ===== Publishers =====
#         self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # ===== Subscribers =====
#         self.imu_sub = self.create_subscription(Float64, '/imu/yaw', self.imu_callback, 10)
#         self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

#         # ===== Joint Names =====
#         self.steer_joints = [
#             "front_left_steer_joint",
#             "front_right_steer_joint",
#             "rear_left_steer_joint",
#             "rear_right_steer_joint"
#         ]
#         self.wheel_joints = [
#             "front_left_wheel_joint",
#             "front_right_wheel_joint",
#             "rear_left_wheel_joint",
#             "rear_right_wheel_joint"
#         ]

#         # ===== Serial =====
#         try:
#             self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.02)
#             self.serial_port.reset_input_buffer()
#             self.serial_port.reset_output_buffer()
#             self.get_logger().info('Serial port opened: /dev/ttyACM0 (115200)')
#         except serial.SerialException as e:
#             self.get_logger().error(f'Failed to open serial port: {e}')
#             raise e

#         # ===== State =====
#         self.wheel_rpm = [0.0, 0.0, 0.0, 0.0]
#         self.steer_angle = [0.0, 0.0, 0.0, 0.0]
#         self.wheel_pos = [0.0, 0.0, 0.0, 0.0]

#         self.prev_time = self.get_clock().now()

#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.theta_imu = 0.0
#         self.prev_theta_for_omega = None

#         self.prev_theta_imu_raw = None
#         self.theta_imu_unwrapped = 0.0

#         # Robot geometry
#         self.lx = [0.35, 0.35, -0.35, -0.35]
#         self.ly = [0.15, -0.15, 0.15, -0.15]
#         self.wheel_radius = 0.065

#         self.vx_modules = [0.0] * 4
#         self.vy_modules = [0.0] * 4

#         # ===== Serial packet format =====
#         # RX packet: STX(3) + hhhhffff + ETX(2)
#         self.STX = b'STX'
#         self.ETX = b'\r\n'
#         self.rx_packet_size = len(self.STX) + 4 * 2 + 4 * 4 + len(self.ETX)  # 29 bytes

#         # TX safety bits
#         self.alive_bit = 0
#         self.estop_bit = 0

#         # Latest command
#         self.last_cmd_vx = 0.0
#         self.last_cmd_vy = 0.0
#         self.last_cmd_w = 0.0
#         self.last_cmd_time = self.get_clock().now()

#         # command timeout
#         self.cmd_timeout_sec = 0.3

#         # feedback timestamp
#         self.last_feedback_time = self.get_clock().now()
#         self.feedback_timeout_sec = 0.2

#         # debug counters
#         self.loop_count = 0
#         self.rx_fail_count = 0
#         self.rx_success_count = 0

#         # ===== Timer =====
#         self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

#         self.get_logger().info('OdomCmdvelSerialNode initialized')

#     def wrap_angle(self, angle):
#         return ((angle + math.pi) % (2.0 * math.pi)) - math.pi

#     def clamp(self, x, lo, hi):
#         return max(lo, min(hi, x))

#     # ===== IMU callback =====
#     def imu_callback(self, msg):
#         theta_raw = msg.data * math.pi / 180.0

#         if self.prev_theta_imu_raw is None:
#             self.prev_theta_imu_raw = theta_raw
#             self.theta_imu_unwrapped = theta_raw
#             self.theta_imu = theta_raw
#             self.get_logger().info(
#                 f'[IMU] first yaw received: raw_deg={msg.data:.2f}, raw_rad={theta_raw:.3f}'
#             )
#             return

#         diff = theta_raw - self.prev_theta_imu_raw
#         if diff > math.pi:
#             diff -= 2.0 * math.pi
#         elif diff < -math.pi:
#             diff += 2.0 * math.pi

#         self.theta_imu_unwrapped += diff
#         self.prev_theta_imu_raw = theta_raw
#         self.theta_imu = self.theta_imu_unwrapped

#     # ===== cmd_vel callback =====
#     def cmd_vel_callback(self, msg):
#         self.last_cmd_vx = msg.linear.x
#         self.last_cmd_vy = msg.linear.y
#         self.last_cmd_w = msg.angular.z
#         self.last_cmd_time = self.get_clock().now()

#         self.get_logger().info(
#             f'[CMD_VEL] vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, wz={msg.angular.z:.3f}'
#         )

#     def _to_s16(self, x: float) -> int:
#         if math.isnan(x) or math.isinf(x):
#             x = 0.0
#         i = int(round(x))
#         if i > 32767:
#             i = 32767
#         if i < -32768:
#             i = -32768
#         return i

#     # ===== TX serial =====
#     def send_serial_cmd(self, vx, vy, omega):
#         # clamp first, do NOT zero-out when exceeded
#         vx = self.clamp(vx, -0.6, 0.6)
#         vy = self.clamp(vy, -0.6, 0.6)
#         omega = self.clamp(omega, -1.2, 1.2)

#         vx_i = self._to_s16(vx * 100.0)
#         vy_i = self._to_s16(vy * 100.0)

#         # NOTE:
#         # If rotation direction is opposite, remove the minus sign below.
#         w_i = self._to_s16(-(omega * 1000.0))

#         self.alive_bit ^= 1
#         safety = ((self.alive_bit & 1) << 1) | (self.estop_bit & 1)

#         packet = bytearray(12)
#         packet[0:3] = self.STX
#         packet[3:5] = vx_i.to_bytes(2, 'big', signed=True)
#         packet[5:7] = vy_i.to_bytes(2, 'big', signed=True)
#         packet[7:9] = w_i.to_bytes(2, 'big', signed=True)
#         packet[9] = safety
#         packet[10] = 0x0D
#         packet[11] = 0x0A

#         try:
#             self.serial_port.write(packet)
#             self.get_logger().debug(
#                 f'[TX] vx={vx:.3f}, vy={vy:.3f}, wz={omega:.3f} | '
#                 f'vx_i={vx_i}, vy_i={vy_i}, w_i={w_i}, safety={safety}'
#             )
#         except serial.SerialException as e:
#             self.get_logger().error(f'Serial write failed: {e}')

#     # ===== RX serial =====
#     def read_serial_feedback(self):
#         try:
#             waiting = self.serial_port.in_waiting
#         except serial.SerialException as e:
#             self.get_logger().error(f'Serial read failed (in_waiting): {e}')
#             return False

#         if waiting < self.rx_packet_size:
#             self.get_logger().warn(
#                 f'[RX] insufficient bytes: waiting={waiting}, need={self.rx_packet_size}'
#             )
#             return False

#         try:
#             buf = self.serial_port.read(waiting)
#         except serial.SerialException as e:
#             self.get_logger().error(f'Serial read failed: {e}')
#             return False

#         self.get_logger().debug(f'[RX] read {len(buf)} bytes')

#         # Use the latest complete packet, not the first one
#         last_valid_packet = None
#         search_start = 0
#         valid_count = 0

#         while True:
#             stx_idx = buf.find(self.STX, search_start)
#             if stx_idx == -1:
#                 break

#             end_idx = stx_idx + self.rx_packet_size
#             if end_idx <= len(buf):
#                 packet = buf[stx_idx:end_idx]
#                 if packet[-2:] == self.ETX:
#                     last_valid_packet = packet
#                     valid_count += 1

#             search_start = stx_idx + 1

#         if last_valid_packet is None:
#             self.get_logger().warn(
#                 f'[RX] no valid packet found in {len(buf)} bytes buffer'
#             )
#             return False

#         self.get_logger().debug(f'[RX] valid packets found: {valid_count}')

#         payload = last_valid_packet[len(self.STX):-len(self.ETX)]

#         try:
#             rpm1, rpm2, rpm3, rpm4, ang1, ang2, ang3, ang4 = struct.unpack('<hhhhffff', payload)
#         except struct.error as e:
#             self.get_logger().warn(f'[RX] struct unpack error: {e}')
#             return False

#         self.wheel_rpm = [
#             rpm1 * 0.1 if rpm1 != 0 else 0.0,
#             rpm2 * 0.1 if rpm2 != 0 else 0.0,
#             rpm3 * 0.1 if rpm3 != 0 else 0.0,
#             rpm4 * 0.1 if rpm4 != 0 else 0.0
#         ]

#         # MCU가 rad를 보내는 경우
#         self.steer_angle = [
#             self.wrap_angle(ang1),
#             self.wrap_angle(ang2),
#             self.wrap_angle(ang3),
#             self.wrap_angle(ang4)
#         ]

#         # 만약 MCU가 degree를 보낸다면 위 대신 아래를 사용해야 함:
#         # self.steer_angle = [
#         #     self.wrap_angle(math.radians(ang1)),
#         #     self.wrap_angle(math.radians(ang2)),
#         #     self.wrap_angle(math.radians(ang3)),
#         #     self.wrap_angle(math.radians(ang4))
#         # ]

#         self.last_feedback_time = self.get_clock().now()
#         self.rx_success_count += 1

#         self.get_logger().info(
#             f'[RX OK] rpm={self.wheel_rpm} '
#             f'ang={[round(a, 3) for a in self.steer_angle]} '
#             f'raw_ang={[round(ang1,3), round(ang2,3), round(ang3,3), round(ang4,3)]}'
#         )

#         return True

#     def set_estop(self, state: bool):
#         self.estop_bit = 1 if state else 0
#         self.get_logger().info(f"ESTOP set to {'ON' if state else 'OFF'}")

#     # ===== Main loop =====
#     def loop(self):
#         self.loop_count += 1

#         current_time = self.get_clock().now()
#         dt = (current_time - self.prev_time).nanoseconds / 1e9
#         if dt <= 0.0:
#             return
#         self.prev_time = current_time

#         # 1) send latest cmd_vel
#         cmd_age = (current_time - self.last_cmd_time).nanoseconds / 1e9
#         if cmd_age > self.cmd_timeout_sec:
#             vx_cmd = 0.0
#             vy_cmd = 0.0
#             w_cmd = 0.0
#             self.get_logger().warn(
#                 f'[CMD TIMEOUT] age={cmd_age:.3f}s -> sending zero cmd'
#             )
#         else:
#             vx_cmd = self.last_cmd_vx
#             vy_cmd = self.last_cmd_vy
#             w_cmd = self.last_cmd_w

#         self.send_serial_cmd(vx_cmd, vy_cmd, w_cmd)

#         # 2) read feedback
#         updated = self.read_serial_feedback()
#         if not updated:
#             self.rx_fail_count += 1
#             self.get_logger().warn(
#                 f'[RX FAIL] fail_count={self.rx_fail_count}, success_count={self.rx_success_count}'
#             )

#         fb_age = (current_time - self.last_feedback_time).nanoseconds / 1e9
#         feedback_stale = fb_age > self.feedback_timeout_sec

#         if feedback_stale:
#             self.get_logger().warn(
#                 f'[FEEDBACK STALE] age={fb_age:.3f}s -> using zero wheel velocity'
#             )
#             wheel_velocities = [0.0, 0.0, 0.0, 0.0]
#         else:
#             wheel_velocities = [rpm * math.pi / 30.0 for rpm in self.wheel_rpm]  # rad/s

#         # 3) Joint state
#         for i in range(4):
#             self.wheel_pos[i] += wheel_velocities[i] * dt

#         joint_state_msg = JointState()
#         joint_state_msg.header.stamp = current_time.to_msg()
#         joint_state_msg.name = self.steer_joints + self.wheel_joints
#         joint_state_msg.position = list(self.steer_angle) + list(self.wheel_pos)
#         joint_state_msg.velocity = [0.0] * 4 + wheel_velocities
#         joint_state_msg.effort = []
#         self.joint_states_pub.publish(joint_state_msg)

#         # 4) Odometry from wheel feedback
#         for i in range(4):
#             v = wheel_velocities[i] * self.wheel_radius
#             self.vx_modules[i] = v * math.cos(self.steer_angle[i])
#             self.vy_modules[i] = v * math.sin(self.steer_angle[i])

#         vx = sum(self.vx_modules) / 4.0
#         vy = sum(self.vy_modules) / 4.0

#         numerator = sum(
#             self.lx[i] * self.vy_modules[i] - self.ly[i] * self.vx_modules[i]
#             for i in range(4)
#         )
#         denominator = sum(self.lx[i] ** 2 + self.ly[i] ** 2 for i in range(4))
#         omega_wheel = numerator / denominator if denominator > 1e-9 else 0.0

#         # Use IMU heading directly for pose integration
#         prev_theta = self.theta
#         self.theta = self.theta_imu

#         if self.prev_theta_for_omega is None:
#             omega_imu = 0.0
#             self.prev_theta_for_omega = self.theta
#         else:
#             omega_imu = (self.theta - self.prev_theta_for_omega) / dt
#             self.prev_theta_for_omega = self.theta

#         delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
#         delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

#         self.x += delta_x
#         self.y += delta_y

#         # 5) Publish odom
#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_footprint'

#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0
#         odom_msg.pose.pose.orientation.x = 0.0
#         odom_msg.pose.pose.orientation.y = 0.0
#         odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
#         odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

#         odom_msg.twist.twist.linear.x = vx
#         odom_msg.twist.twist.linear.y = vy
#         odom_msg.twist.twist.angular.z = omega_imu

#         self.odom_pub.publish(odom_msg)

#         # 6) Publish TF
#         t = TransformStamped()
#         t.header.stamp = current_time.to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_footprint'
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0
#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = 0.0
#         t.transform.rotation.z = math.sin(self.theta / 2.0)
#         t.transform.rotation.w = math.cos(self.theta / 2.0)
#         self.tf_broadcaster.sendTransform(t)

#         # 7) Periodic debug print
#         if self.loop_count % 25 == 0:  # about 0.5 sec at 50Hz
#             self.get_logger().info(
#                 f'[LOOP] dt={dt:.3f} '
#                 f'cmd=({vx_cmd:.3f}, {vy_cmd:.3f}, {w_cmd:.3f}) '
#                 f'fb_age={fb_age:.3f}s stale={feedback_stale} '
#                 f'vx={vx:.3f} vy={vy:.3f} '
#                 f'omega_wheel={omega_wheel:.3f} omega_imu={omega_imu:.3f} '
#                 f'pose=({self.x:.3f}, {self.y:.3f}, {self.theta:.3f})'
#             )


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomCmdvelSerialNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Stopped by user')
#     finally:
#         if hasattr(node, 'serial_port') and node.serial_port.is_open:
#             node.serial_port.close()
#             node.get_logger().info('Serial port closed')
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()






import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster

import serial
import struct
import math


class OdomCmdvelSerialNode(Node):
    def __init__(self):
        super().__init__('odom_cmdvel_serial_node')

        # ===== Publishers =====
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== Subscribers =====
        self.imu_sub = self.create_subscription(Float64, '/imu/yaw', self.imu_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ===== Joint Names =====
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

        # ===== Serial =====
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.02)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise e

        # ===== State =====
        self.wheel_rpm = [0.0, 0.0, 0.0, 0.0]
        self.steer_angle = [0.0, 0.0, 0.0, 0.0]
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]

        self.prev_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.theta_imu = 0.0
        self.prev_theta_for_omega = None

        self.prev_theta_imu_raw = None
        self.theta_imu_unwrapped = 0.0

        # Robot geometry
        self.lx = [0.35, 0.35, -0.35, -0.35]
        self.ly = [0.15, -0.15, 0.15, -0.15]
        self.wheel_radius = 0.065

        self.vx_modules = [0.0] * 4
        self.vy_modules = [0.0] * 4

        # ===== Serial packet format =====
        self.STX = b'STX'
        self.ETX = b'\r\n'
        self.rx_packet_size = len(self.STX) + 4 * 2 + 4 * 4 + len(self.ETX)

        # TX safety bits
        self.alive_bit = 0
        self.estop_bit = 0

        # Latest command
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        # command timeout
        self.cmd_timeout_sec = 0.3

        # feedback timestamp
        self.last_feedback_time = self.get_clock().now()
        self.feedback_timeout_sec = 0.2

        # ===== Timer =====
        self.timer = self.create_timer(0.02, self.loop)

    def wrap_angle(self, angle):
        return ((angle + math.pi) % (2.0 * math.pi)) - math.pi

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def imu_callback(self, msg):
        theta_raw = msg.data * math.pi / 180.0

        if self.prev_theta_imu_raw is None:
            self.prev_theta_imu_raw = theta_raw
            self.theta_imu_unwrapped = theta_raw
            self.theta_imu = theta_raw
            return

        diff = theta_raw - self.prev_theta_imu_raw
        if diff > math.pi:
            diff -= 2.0 * math.pi
        elif diff < -math.pi:
            diff += 2.0 * math.pi

        self.theta_imu_unwrapped += diff
        self.prev_theta_imu_raw = theta_raw
        self.theta_imu = self.theta_imu_unwrapped

    def cmd_vel_callback(self, msg):
        self.last_cmd_vx = msg.linear.x
        self.last_cmd_vy = msg.linear.y
        self.last_cmd_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _to_s16(self, x: float) -> int:
        if math.isnan(x) or math.isinf(x):
            x = 0.0
        i = int(round(x))
        if i > 32767:
            i = 32767
        if i < -32768:
            i = -32768
        return i

    def send_serial_cmd(self, vx, vy, omega):
        vx = self.clamp(vx, -0.6, 0.6)
        vy = self.clamp(vy, -0.6, 0.6)
        omega = self.clamp(omega, -1.2, 1.2)

        vx_i = self._to_s16(vx * 100.0)
        vy_i = self._to_s16(vy * 100.0)
        w_i = self._to_s16(-(omega * 1000.0))

        self.alive_bit ^= 1
        safety = ((self.alive_bit & 1) << 1) | (self.estop_bit & 1)

        packet = bytearray(12)
        packet[0:3] = self.STX
        packet[3:5] = vx_i.to_bytes(2, 'big', signed=True)
        packet[5:7] = vy_i.to_bytes(2, 'big', signed=True)
        packet[7:9] = w_i.to_bytes(2, 'big', signed=True)
        packet[9] = safety
        packet[10] = 0x0D
        packet[11] = 0x0A

        try:
            self.serial_port.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def read_serial_feedback(self):
        try:
            waiting = self.serial_port.in_waiting
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed (in_waiting): {e}')
            return False

        if waiting < self.rx_packet_size:
            return False

        try:
            buf = self.serial_port.read(waiting)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed: {e}')
            return False

        last_valid_packet = None
        search_start = 0

        while True:
            stx_idx = buf.find(self.STX, search_start)
            if stx_idx == -1:
                break

            end_idx = stx_idx + self.rx_packet_size
            if end_idx <= len(buf):
                packet = buf[stx_idx:end_idx]
                if packet[-2:] == self.ETX:
                    last_valid_packet = packet

            search_start = stx_idx + 1

        if last_valid_packet is None:
            return False

        payload = last_valid_packet[len(self.STX):-len(self.ETX)]

        try:
            rpm1, rpm2, rpm3, rpm4, ang1, ang2, ang3, ang4 = struct.unpack('<hhhhffff', payload)
        except struct.error:
            return False

        self.wheel_rpm = [
            rpm1 * 0.1 if rpm1 != 0 else 0.0,
            rpm2 * 0.1 if rpm2 != 0 else 0.0,
            rpm3 * 0.1 if rpm3 != 0 else 0.0,
            rpm4 * 0.1 if rpm4 != 0 else 0.0
        ]

        self.steer_angle = [
            self.wrap_angle(ang1),
            self.wrap_angle(ang2),
            self.wrap_angle(ang3),
            self.wrap_angle(ang4)
        ]

        self.last_feedback_time = self.get_clock().now()
        return True

    def set_estop(self, state: bool):
        self.estop_bit = 1 if state else 0

    def loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.prev_time = current_time

        cmd_age = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if cmd_age > self.cmd_timeout_sec:
            vx_cmd = 0.0
            vy_cmd = 0.0
            w_cmd = 0.0
        else:
            vx_cmd = self.last_cmd_vx
            vy_cmd = self.last_cmd_vy
            w_cmd = self.last_cmd_w

        self.send_serial_cmd(vx_cmd, -vy_cmd, w_cmd)

        updated = self.read_serial_feedback()

        fb_age = (current_time - self.last_feedback_time).nanoseconds / 1e9
        feedback_stale = fb_age > self.feedback_timeout_sec

        if feedback_stale:
            wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        else:
            wheel_velocities = [rpm * math.pi / 30.0 for rpm in self.wheel_rpm]

        for i in range(4):
            self.wheel_pos[i] += wheel_velocities[i] * dt

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.name = self.steer_joints + self.wheel_joints
        joint_state_msg.position = list(self.steer_angle) + list(self.wheel_pos)
        joint_state_msg.velocity = [0.0] * 4 + wheel_velocities
        joint_state_msg.effort = []
        self.joint_states_pub.publish(joint_state_msg)

        for i in range(4):
            v = wheel_velocities[i] * self.wheel_radius
            self.vx_modules[i] = v * math.cos(self.steer_angle[i])
            self.vy_modules[i] = v * math.sin(self.steer_angle[i])

        vx = sum(self.vx_modules) / 4.0
        vy = sum(self.vy_modules) / 4.0

        numerator = sum(
            self.lx[i] * self.vy_modules[i] - self.ly[i] * self.vx_modules[i]
            for i in range(4)
        )
        denominator = sum(self.lx[i] ** 2 + self.ly[i] ** 2 for i in range(4))
        omega_wheel = numerator / denominator if denominator > 1e-9 else 0.0

        self.theta = self.theta_imu

        if self.prev_theta_for_omega is None:
            omega_imu = 0.0
            self.prev_theta_for_omega = self.theta
        else:
            omega_imu = (self.theta - self.prev_theta_for_omega) / dt
            self.prev_theta_for_omega = self.theta

        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

        self.x += delta_x
        self.y += delta_y

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega_imu

        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCmdvelSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'serial_port') and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()