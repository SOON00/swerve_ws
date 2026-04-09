import sys
import select
import termios
import tty
import rclpy
from geometry_msgs.msg import Twist

# 속도 스텝
LIN_VEL_STEP = 0.05  # x, y 방향 속도 스텝
ANG_VEL_STEP = 0.1   # 회전 속도 스텝

# 최대/최소 속도 제한
LIN_MAX = 2.5
LIN_MIN = -2.5
ANG_MAX = 1.0
ANG_MIN = -1.0

msg = """
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

w/x : +x / -x linear velocity
a/d : -y / +y linear velocity
q/e : -omega / +omega angular velocity
s : stop
CTRL-C to quit
"""

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    vx = 0.0
    vy = 0.0
    omega = 0.0

    print(msg)

    try:
        while True:
            key = get_key()

            if key == 'w':
                vx += LIN_VEL_STEP
            elif key == 'x':
                vx -= LIN_VEL_STEP
            elif key == 'a':
                vy -= LIN_VEL_STEP
            elif key == 'd':
                vy += LIN_VEL_STEP
            elif key == 'q':
                omega -= ANG_VEL_STEP
            elif key == 'e':
                omega += ANG_VEL_STEP
            elif key == 's':
                vx = 0.0
                vy = 0.0
                omega = 0.0
            elif key == '\x03':  # CTRL-C
                break

            vx = constrain(vx, LIN_MIN, LIN_MAX)
            vy = constrain(vy, LIN_MIN, LIN_MAX)
            omega = constrain(omega, ANG_MIN, ANG_MAX)

            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = omega
            pub.publish(twist)

    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
