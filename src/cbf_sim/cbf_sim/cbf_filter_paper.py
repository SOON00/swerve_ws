import rclpy
from rclpy.node import Node

import numpy as np
import cvxpy as cp
import yaml
from pathlib import Path

from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


# ================= YAML Loader =================
def load_cbf_map():
    pkg_share = get_package_share_directory("cbf_sim")
    yaml_path = Path(pkg_share) / "params" / "cbf_params.yaml"

    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)


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
    return lines, Pc2, Pc3


# ================= CBF =================
def line_cbf(p, line):
    return line["a"]*p[0] + line["b"]*p[1] + line["c"]


def hi(x, i, L, dl, db, Pc2, Pc3, turn_dir, lines):
    xr, yr, th = x

    if turn_dir == 1:
        P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                    yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
        P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                    yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                    yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                    yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
    elif turn_dir == -1:
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
        (P4[0]-P3[0])*(p[1]-P3[1]) -
        (P4[1]-P3[1])*(p[0]-P3[0])
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


# ================= ROS2 Node =================
class BodyFrameCBF(Node):
    def __init__(self):
        super().__init__('cbf_filter_paper')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        data = load_cbf_map()
        
        # Map Parameter
        self.lines, self.Pc2, self.Pc3 = parse_cbf_yaml(data)
        self.turn = 1
        
        # Start & Goal Pose
        self.x = np.array([-4.0, -1.5, np.pi/2])
        self.G = np.array([-1.5, 3.0, 0.0])

        self.dt = 0.05
        
        # Robot Parameter
        self.L, self.dl, self.db = 3.0, 0.25, 0.35
        self.v_max, self.w_max = 2.5, 2.5
        
        # Gain
        self.p1 = 0.5
        self.p2 = 0.5
        self.p3 = 0.5
        self.k_cbf = 0.1

        self.timer = self.create_timer(self.dt, self.loop)

    def loop(self):
        # Coordinate Rotation Transform (CW)
        th = self.x[2]
        R = np.array([
            [np.cos(th), -np.sin(th), 0],
            [np.sin(th),  np.cos(th), 0],
            [0,           0,          1]
        ])

        # Nominal Controller
        u_nom_w = -np.array([
            self.p1*(self.x[0]-self.G[0]),
            self.p2*(self.x[1]-self.G[1]),
            self.p3*(self.x[2]-self.G[2])
        ])
        u_nom = np.linalg.solve(R, u_nom_w) # World -> Odom Coordinate

        u = cp.Variable(3)
        cost = 0.5*cp.sum_squares(u) - u_nom @ u
        cons = []

        for i in range(6):
            h, dhdx = cbf_i(
                self.x, i+1,
                self.L, self.dl, self.db,
                self.Pc2, self.Pc3,
                self.turn, self.lines
            )
            cons.append(-dhdx @ (R @ u) <= self.k_cbf*h)

        cons += [
            u[0] <= self.v_max, u[0] >= -self.v_max,
            u[1] <= self.v_max, u[1] >= -self.v_max,
            u[2] <= self.w_max, u[2] >= -self.w_max
        ]

        cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)

        if u.value is None:
            return

        # State Update (Odom -> World coordinate)
        self.x = self.x + self.dt * (R @ u.value)

        # cmd_vel
        twist = Twist()
        twist.linear.x = u.value[0]
        twist.linear.y = u.value[1]
        twist.angular.z = u.value[2]
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BodyFrameCBF())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
