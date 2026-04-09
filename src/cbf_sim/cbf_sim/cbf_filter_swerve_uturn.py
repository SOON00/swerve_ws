import rclpy
from rclpy.node import Node

import numpy as np
import cvxpy as cp
import yaml
from pathlib import Path

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

# ================= Utils =================
def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ================= CBF Map Loader =================
def load_cbf_segments():
    pkg_share = get_package_share_directory("cbf_sim")
    yaml_path = Path(pkg_share) / "params" / "uturn_params.yaml"

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    segments = []
    for seg in data["goals"]:
        lines, Pc2, Pc3, turn_dir = parse_cbf_yaml(seg)
        segments.append({
            "start": np.array(seg["start"]),
            "goal":  np.array(seg["goal"]),
            "lines": lines,
            "Pc2": Pc2,
            "Pc3": Pc3,
            "turn": turn_dir
        })
    return segments

def make_lines(a1, b1, c1, a2, b2, c2):
    return {
        "Lc1": {"a": a1, "b": b1, "c": c1},
        "Lc2": {"a": a2, "b": b2, "c": c2},
    }

def parse_cbf_yaml(data):
    l1 = data["cbf_lines"]["Lc1"]
    l2 = data["cbf_lines"]["Lc2"]

    lines = make_lines(
        l1["a"], l1["b"], l1["c"],
        l2["a"], l2["b"], l2["c"]
    )

    Pc2 = np.array(data["corner_points"]["Pc2"])
    Pc3 = np.array(data["corner_points"]["Pc3"])
    turn_dir = data["turn_dir"]

    return lines, Pc2, Pc3, turn_dir

# ================= CBF =================
def line_cbf(p, line):
    return line["a"] * p[0] + line["b"] * p[1] + line["c"]

def hi(x, i, L, dl, db, Pc2, Pc3, turn_dir, lines):
    xr, yr, th = x

    if turn_dir == 1:  # Turn Right
        P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
        P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
    else:  # Turn Left
        P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
        P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])

    if i == 1: return -line_cbf(P1, lines["Lc1"])
    if i == 2: return -line_cbf(P2, lines["Lc1"])
    if i == 3: return -line_cbf(P1, lines["Lc2"])
    if i == 4: return -line_cbf(P2, lines["Lc2"])

    Lr = lambda p: (turn_dir) * (
        (P4[0] - P3[0]) * (p[1] - P3[1]) -
        (P4[1] - P3[1]) * (p[0] - P3[0])
    ) / np.linalg.norm(P4 - P3)

    if i == 5: return -Lr(Pc2)
    if i == 6: return -Lr(Pc3)

def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn_dir, lines):
    eps = 1e-4
    h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn_dir, lines)
    dhdx = np.zeros(3)
    for j in range(3):
        xp = x.copy()
        xp[j] += eps
        dhdx[j] = (hi(xp, i, L, dl, db, Pc2, Pc3, turn_dir, lines) - h0) / eps
    return h0, dhdx

# ================= Node =================
class MultiGoalCBF(Node):
    def __init__(self):
        super().__init__('cbf_filter_multi_goal')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_cbf', 10)
        self.cbf_active_pub = self.create_publisher(Bool, '/cbf_active', 10)
        self.cbf_enable_pub = self.create_publisher(Bool, '/cbf_enable', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load all segments
        self.segments = load_cbf_segments()
        self.current_idx = 0
        self.x = self.segments[0]["start"].copy()

        # Vehicle parameters
        self.dt = 0.01
        self.L = 0.8
        self.dl = 0.1
        self.db = 0.4
        self.v_max, self.w_max = 2.5, 2.5
        self.p1 = self.p2 = self.p3 = 0.5
        self.k_cbf = 0.1

        self.start_tol_pos = 0.1
        self.start_tol_ang = 5*np.pi/180
        self.goal_tol_pos  = 0.1
        self.goal_tol_ang  = 1*np.pi/180

        self.cbf_enabled = False

        # Timer
        self.timer = self.create_timer(self.dt, self.loop)
        
        self.param_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        while not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /controller_server/set_parameters service...")
            
    def set_heading_lock(self, value):
        param = Parameter()
        param.name = "FollowPath.HeadingLockCritic.locked_heading"
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)

        req = SetParameters.Request()
        req.parameters = [param]
        future = self.param_cli.call_async(req)
        return future

    def publish_bool(self, pub, value):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def update_state_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "odom", "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            self.x = np.array([t.x, t.y, yaw_from_quaternion(q)])
            return True
        except Exception:
            return False

    def loop(self):
        if not self.update_state_from_tf():
            self.publish_bool(self.cbf_active_pub, False)
            self.publish_bool(self.cbf_enable_pub, False)
            return

        # ================= Current Segment =================
        seg = self.segments[self.current_idx]
        G = seg["goal"]
        lines, Pc2, Pc3, turn = seg["lines"], seg["Pc2"], seg["Pc3"], seg["turn"]

        # ========= Entry =========
        if not self.cbf_enabled:
            pos_err_s = np.linalg.norm(self.x[:2] - seg["start"][:2])
            ang_err_s = abs(wrap_to_pi(self.x[2] - seg["start"][2]))

            if pos_err_s < self.start_tol_pos and ang_err_s < self.start_tol_ang:
                self.cbf_enabled = True
                self.get_logger().info(f"CBF Entered Goal {self.current_idx}")
                self.publish_bool(self.cbf_enable_pub, True)
            else:
                self.publish_bool(self.cbf_active_pub, False)
                self.publish_bool(self.cbf_enable_pub, False)
                return

        self.publish_bool(self.cbf_active_pub, True)

        # ========= Exit =========
        pos_err_g = np.linalg.norm(self.x[:2] - G[:2])
        ang_err_g = abs(wrap_to_pi(self.x[2] - G[2]))

        if pos_err_g < self.goal_tol_pos and ang_err_g < self.goal_tol_ang:
            self.set_heading_lock(self.x[2])
            self.cbf_enabled = False
            self.publish_bool(self.cbf_active_pub, False)
            self.publish_bool(self.cbf_enable_pub, False)
            self.get_logger().info(f"CBF Exited Goal {self.current_idx}")
            # Move to next goal
            if self.current_idx + 1 < len(self.segments):
                self.current_idx += 1
                self.get_logger().info(f"Switching to Goal {self.current_idx}")
            return

        # ========= Nominal =========
        th = self.x[2]
        R = np.array([
            [np.cos(th), -np.sin(th), 0],
            [np.sin(th),  np.cos(th), 0],
            [0, 0, 1]
        ])

        u_nom_w = -np.array([
            self.p1 * (self.x[0] - G[0]),
            self.p2 * (self.x[1] - G[1]),
            self.p3 * wrap_to_pi(self.x[2] - G[2])
        ])
        u_nom = np.linalg.solve(R, u_nom_w)

        # ========= QP =========
        u = cp.Variable(3)
        cost = 0.5 * cp.sum_squares(u) - u_nom @ u

        cons = []
        for i in range(6):
            h, dhdx = cbf_i(
                self.x, i + 1,
                self.L, self.dl, self.db,
                Pc2, Pc3, turn, lines
            )
            cons.append(-dhdx @ (R @ u) <= self.k_cbf * h)

        cons += [
            u[0] <= self.v_max, u[0] >= -self.v_max,
            u[1] <= self.v_max, u[1] >= -self.v_max,
            u[2] <= self.w_max, u[2] >= -self.w_max,
        ]

        cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)

        if u.value is None:
            return

        twist = Twist()
        twist.linear.x  = float(u.value[0])
        twist.linear.y  = float(u.value[1])
        twist.angular.z = float(u.value[2])
        self.cmd_pub.publish(twist)

# ================= Main =================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MultiGoalCBF())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
