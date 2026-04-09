import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class CmdvelSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_serial')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        try:
            self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
            self.get_logger().info("Serial connected to /dev/ttyACM0")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port /dev/ttyACM0: {e}")
            self.serial = None

        # SAFETY bits
        self.alive_bit = 0
        self.estop_bit = 0

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        self.send_serial(linear_x, linear_y, angular_z)

    def _to_s16(self, x: float) -> int:
        if math.isnan(x) or math.isinf(x):
            x = 0.0
        i = int(round(x))
        if i > 32767:
            i = 32767
        if i < -32768:
            i = -32768
        return i

    def send_serial(self, vx, vy, omega):
        if self.serial is None:
            return

        vx_i = self._to_s16(vx * 100.0)
        vy_i = self._to_s16(vy * 100.0)
        w_i  = self._to_s16(-(omega * 1000.0))
        
        if abs(vx_i) > 60:
            vx_i = 0
        if abs(vy_i) > 60:
            vy_i = 0
        if abs(w_i) > 1200:
            w_i = 0

        self.alive_bit ^= 1
        safety = ((self.alive_bit & 1) << 1) | (self.estop_bit & 1)

        packet = bytearray(12)
        packet[0:3] = b'STX'
        packet[3:5] = vx_i.to_bytes(2, 'big', signed=True)
        packet[5:7] = vy_i.to_bytes(2, 'big', signed=True)
        packet[7:9] = w_i.to_bytes(2, 'big', signed=True)
        packet[9]   = safety
        packet[10]  = 0x0D
        packet[11]  = 0x0A

        try:
            self.serial.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def set_estop(self, state: bool):
        self.estop_bit = 1 if state else 0
        self.get_logger().info(f"ESTOP set to {'ON' if state else 'OFF'}")


def main(args=None):
    rclpy.init(args=args)
    controller = CmdvelSerial()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    if controller.serial:
        controller.serial.close()

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()