import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class SwerveDriveController(Node):
    def __init__(self):
        super().__init__('swerve_drive_controller')
        
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.steering_pub = self.create_publisher(Float64MultiArray, '/swerve_steering_controller/commands', 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/swerve_velocity_controller/commands', 10)
        
        self.wheel_radius = 0.065
        self.wheel_base = 0.34
        self.track_width = 0.30
        
        self.wheel_positions = {
            'front_left':  ( self.wheel_base/2,  self.track_width/2),
            'front_right': ( self.wheel_base/2, -self.track_width/2),
            'rear_left':   (-self.wheel_base/2,  self.track_width/2),
            'rear_right':  (-self.wheel_base/2, -self.track_width/2),
        }
        
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        wheel_commands = self.calc_kinematics(linear_x, linear_y, angular_z)
        
        self.publish_commands(wheel_commands)
    
    def calc_kinematics(self, v_bx, v_by, w_bz):
        speeds = []
        angles = []
        
        for wheel, (l_ix, l_iy) in self.wheel_positions.items():
            v_ix = v_bx - w_bz * l_iy
            v_iy = v_by + w_bz * l_ix
            
            speed = math.sqrt(v_ix**2 + v_iy**2) / self.wheel_radius
            angle = math.atan2(v_iy, v_ix)
            
            speeds.append(speed)
            angles.append(angle)
            
        return {
            'angles': angles,  # [FL, FR, RL, RR]
            'speeds': speeds
        }
    
    def publish_commands(self, wheel_commands):
        steering_msg = Float64MultiArray()
        steering_msg.data = wheel_commands['angles']
        self.steering_pub.publish(steering_msg)
        
        velocity_msg = Float64MultiArray()
        velocity_msg.data = wheel_commands['speeds']
        self.velocity_pub.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SwerveDriveController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
