# import numpy as np
# import cvxpy as cp
# import matplotlib.pyplot as plt
# import yaml
# from pathlib import Path

# # ================= YAML Loader =================
# def load_cbf_map():
#     this_file = Path(__file__).resolve()
#     pkg_root  = this_file.parent.parent
#     yaml_path = pkg_root / "params" / "cbf_heading.yaml"

#     with open(yaml_path, "r") as f:
#         data = yaml.safe_load(f)

#     return data

# def make_lines(a1, b1, c1, a2, b2, c2):
#     return {
#         "Lc1": {"a": a1, "b": b1, "c": c1},
#         "Lc2": {"a": a2, "b": b2, "c": c2},
#     }

# def parse_cbf_yaml(data):
#     l1 = data["cbf_lines"]["Lc1"]
#     l2 = data["cbf_lines"]["Lc2"]

#     lines = make_lines(
#         l1["a"], l1["b"], l1["c"],
#         l2["a"], l2["b"], l2["c"]
#     )

#     Pc2 = np.array(data["corner_points"]["Pc2"])
#     Pc3 = np.array(data["corner_points"]["Pc3"])

#     return lines, Pc2, Pc3

# # ================= CBF Functions =================
# def line_cbf(p, line):
#     return line["a"]*p[0] + line["b"]*p[1] + line["c"]

# def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines):
#     xr, yr, th = x

#     if turn == 1:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
#     else:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])

#     if i == 1: return -line_cbf(P1, lines["Lc1"])
#     if i == 2: return -line_cbf(P2, lines["Lc1"])
#     if i == 3: return -line_cbf(P1, lines["Lc2"])
#     if i == 4: return -line_cbf(P4, lines["Lc2"])

#     Lr = lambda p: turn*((P4[0]-P3[0])*(p[1]-P3[1]) -
#                           (P4[1]-P3[1])*(p[0]-P3[0])) / np.linalg.norm(P4-P3)

#     if i == 5: return -Lr(Pc2)
#     if i == 6: return -Lr(Pc3)

# def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines):
#     eps = 1e-4
#     h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines)
#     dhdx = np.zeros(3)

#     for j in range(3):
#         xp = x.copy()
#         xp[j] += eps
#         dhdx[j] = (hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines) - h0) / eps

#     return h0, dhdx

# def plot_footprint(x, L, dl, db, turn):
#     xr, yr, th = x
#     P = np.zeros((2,4))

#     if turn == 1:
#         P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]
#         P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
#         P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
#         P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
#     else:
#         P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
#         P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
#         P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
#         P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]

#     plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]],
#              color='y', alpha=0.15, edgecolor='k')
#     plt.scatter(P[0,:], P[1,:], c='k', s=25)

# # ================= Main =================
# if __name__ == "__main__":

#     lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

#     turn = 1
#     x = np.array([ 2.0,  0.0, 0.0])
#     G = np.array([ 4.0, -1.5, 0.0])

#     dt, T = 0.05, 10
#     N = int(T/dt)

#     L, dl, db = 0.8, 0.10, 0.37
#     v_max, w_max = 0.6, 2.5     # 0.2 0.25
    
#     p1 = 1.5
#     p2 = 1.5
#     p3 = 1.5
    
#     k_cbf = 1.0

#     traj = np.zeros((3, N))
#     u_traj = np.zeros((3, N))

#     for k in range(N):
#         traj[:,k] = x

#         u_nom = -np.array([
#             p1*(x[0]-G[0]),
#             p2*(x[1]-G[1]),
#             p3*(x[2]-G[2])
#         ])

#         u = cp.Variable(3)
#         cost = 0.5*cp.sum_squares(u) - u_nom @ u
#         cons = []

#         for i in range(6):
#             h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines)
#             cons.append(-dhdx @ u <= k_cbf*h)

#         cons += [
#             u[0] <= v_max, u[0] >= -v_max,
#             u[1] <= v_max, u[1] >= -v_max,
#             u[2] <= w_max, u[2] >= -w_max
#         ]

#         cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
#         u_traj[:,k] = u.value
#         x = x + dt*u.value

#     # ================= Plot Trajectory =================
#     plt.figure(figsize=(8,8))
#     plt.axis("equal")
#     plt.grid()
#     plt.plot(traj[0], traj[1], 'r', linewidth=3)
#     plt.plot(G[0], G[1], 'ro')

#     for k in range(0, N, N//20):
#         plot_footprint(traj[:,k], L, dl, db, turn)
        
#     # Walls
#     # Outer
#     plt.plot([1, 5],[0.5, 0.5],'b',linewidth=1.5) 
#     plt.plot([5, 5],[0.5, -2.5],'b',linewidth=1.5)
      
#     # Inner
#     plt.plot([1, 3],[-0.5, -0.5],'b',linewidth=1.5)
#     plt.plot([3, 3],[-0.5, -3.8],'b',linewidth=1.5)

#     plt.title("Trajectory & Footprints")
#     plt.xlabel("x [m]")
#     plt.ylabel("y [m]")
#     plt.show()

#     # ================= Plot Control Inputs =================
#     t = np.arange(N) * dt
#     plt.figure(figsize=(10,6))
#     plt.plot(t, u_traj[0], label="u_x")
#     plt.plot(t, u_traj[1], label="u_y")
#     plt.plot(t, u_traj[2], label="u_theta")
#     plt.axhline(v_max, linestyle='--', alpha=0.3)
#     plt.axhline(-v_max, linestyle='--', alpha=0.3)
#     plt.axhline(w_max, linestyle='--', alpha=0.3)
#     plt.axhline(-w_max, linestyle='--', alpha=0.3)
#     plt.grid()
#     plt.legend()
#     plt.xlabel("time [s]")
#     plt.ylabel("control input")
#     plt.title("Control Inputs")
#     plt.show()

# import numpy as np
# import cvxpy as cp
# import matplotlib.pyplot as plt
# import yaml
# from pathlib import Path

# # ================= YAML Loader =================
# def load_cbf_map():
#     this_file = Path(__file__).resolve()
#     pkg_root  = this_file.parent.parent
#     yaml_path = pkg_root / "params" / "cbf_heading.yaml"

#     with open(yaml_path, "r") as f:
#         data = yaml.safe_load(f)

#     return data

# def make_lines(a1, b1, c1, a2, b2, c2):
#     return {
#         "Lc1": {"a": a1, "b": b1, "c": c1},
#         "Lc2": {"a": a2, "b": b2, "c": c2},
#     }

# def parse_cbf_yaml(data):
#     l1 = data["cbf_lines"]["Lc1"]
#     l2 = data["cbf_lines"]["Lc2"]

#     lines = make_lines(
#         l1["a"], l1["b"], l1["c"],
#         l2["a"], l2["b"], l2["c"]
#     )

#     Pc2 = np.array(data["corner_points"]["Pc2"])
#     Pc3 = np.array(data["corner_points"]["Pc3"])

#     return lines, Pc2, Pc3

# # ================= CBF Functions =================
# def line_cbf(p, line):
#     return line["a"]*p[0] + line["b"]*p[1] + line["c"]

# def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines):
#     xr, yr, th = x

#     if turn == 1:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
#     else:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])

#     if i == 1: return -line_cbf(P1, lines["Lc1"])
#     if i == 2: return -line_cbf(P2, lines["Lc1"])
#     if i == 3: return -line_cbf(P1, lines["Lc2"])
#     if i == 4: return -line_cbf(P4, lines["Lc2"])

#     # --- 2차 함수(포물선) CBF 정의 ---
#     def Cr(p):

#         # ----- local frame -----
#         ex = (P4 - P2)
#         ex = ex / np.linalg.norm(ex)

#         ey = (P1 - P3)
#         ey = ey / np.linalg.norm(ey)

#         def to_local(pt):
#             d = pt - P3
#             xp = np.dot(d, ex)
#             yp = np.dot(d, ey)
#             return xp, yp

#         # ----- parabola coefficient -----
#         x2, y2 = to_local(P2)
#         x4, y4 = to_local(P4)

#         eps = 1e-8
#         a2 = y2 / (x2**2 + eps)
#         a4 = y4 / (x4**2 + eps)
#         a = 0.5 * (a2 + a4)

#         # ----- evaluate point -----
#         xp, yp = to_local(p)

#         return yp - a * xp**2

#     # i=5, 6은 환경의 코너 포인트가 로봇의 포물선 영역 밖에 있도록 제약
#     if i == 5: return -Cr(Pc2)
#     if i == 6: return -Cr(Pc3)

# def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines):
#     eps = 1e-4
#     h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines)
#     dhdx = np.zeros(3)

#     for j in range(3):
#         xp = x.copy()
#         xp[j] += eps
#         dhdx[j] = (hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines) - h0) / eps

#     return h0, dhdx

# def plot_footprint(x, L, dl, db, turn):
#     xr, yr, th = x
#     P = np.zeros((2,4))

#     if turn == 1:
#         P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]
#         P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
#         P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
#         P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
#     else:
#         P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
#         P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
#         P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
#         P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]

#     plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]],
#              color='y', alpha=0.15, edgecolor='k')
#     plt.scatter(P[0,:], P[1,:], c='k', s=25)

# # ================= Main =================
# if __name__ == "__main__":

#     lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

#     turn = 1
#     x = np.array([ 2.0,  0.0, 0.0])
#     G = np.array([ 4.0, -1.5, 0.0])

#     dt, T = 0.05, 10
#     N = int(T/dt)

#     L, dl, db = 0.8, 0.10, 0.37
#     v_max, w_max = 0.6, 2.5     # 0.2 0.25
    
#     p1 = 1.5
#     p2 = 1.5
#     p3 = 1.5
    
#     k_cbf = 1.0

#     traj = np.zeros((3, N))
#     u_traj = np.zeros((3, N))

#     for k in range(N):
#         traj[:,k] = x

#         u_nom = -np.array([
#             p1*(x[0]-G[0]),
#             p2*(x[1]-G[1]),
#             p3*(x[2]-G[2])
#         ])

#         u = cp.Variable(3)
#         cost = 0.5*cp.sum_squares(u) - u_nom @ u
#         cons = []

#         for i in range(6):
#             h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines)
#             cons.append(-dhdx @ u <= k_cbf*h)

#         cons += [
#             u[0] <= v_max, u[0] >= -v_max,
#             u[1] <= v_max, u[1] >= -v_max,
#             u[2] <= w_max, u[2] >= -w_max
#         ]

#         cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
#         u_traj[:,k] = u.value
#         x = x + dt*u.value

    # # ================= Plot Trajectory =================
    # plt.figure(figsize=(8,8))
    # plt.axis("equal")
    # plt.grid()
    # plt.plot(traj[0], traj[1], 'r', linewidth=3)
    # plt.plot(G[0], G[1], 'ro')

    # for k in range(0, N, N//20):
    #     plot_footprint(traj[:,k], L, dl, db, turn)
        
    # # Walls
    # # Outer
    # plt.plot([1, 5],[0.5, 0.5],'b',linewidth=1.5) 
    # plt.plot([5, 5],[0.5, -2.5],'b',linewidth=1.5)
      
    # # Inner
    # plt.plot([1, 3],[-0.5, -0.5],'b',linewidth=1.5)
    # plt.plot([3, 3],[-0.5, -3.8],'b',linewidth=1.5)

    # plt.title("Trajectory & Footprints")
    # plt.xlabel("x [m]")
    # plt.ylabel("y [m]")
    # plt.show()

#     # ================= Plot Control Inputs =================
#     t = np.arange(N) * dt
#     plt.figure(figsize=(10,6))
#     plt.plot(t, u_traj[0], label="u_x")
#     plt.plot(t, u_traj[1], label="u_y")
#     plt.plot(t, u_traj[2], label="u_theta")
#     plt.axhline(v_max, linestyle='--', alpha=0.3)
#     plt.axhline(-v_max, linestyle='--', alpha=0.3)
#     plt.axhline(w_max, linestyle='--', alpha=0.3)
#     plt.axhline(-w_max, linestyle='--', alpha=0.3)
#     plt.grid()
#     plt.legend()
#     plt.xlabel("time [s]")
#     plt.ylabel("control input")
#     plt.title("Control Inputs")
#     plt.show()

# import numpy as np
# import cvxpy as cp
# import matplotlib.pyplot as plt
# import yaml
# from pathlib import Path

# # ================= YAML Loader =================
# def load_cbf_map():
#     this_file = Path(__file__).resolve()
#     pkg_root  = this_file.parent.parent
#     yaml_path = pkg_root / "params" / "cbf_heading.yaml"

#     with open(yaml_path, "r") as f:
#         data = yaml.safe_load(f)

#     return data

# def make_lines(a1, b1, c1, a2, b2, c2):
#     return {
#         "Lc1": {"a": a1, "b": b1, "c": c1},
#         "Lc2": {"a": a2, "b": b2, "c": c2},
#     }

# def parse_cbf_yaml(data):
#     l1 = data["cbf_lines"]["Lc1"]
#     l2 = data["cbf_lines"]["Lc2"]

#     lines = make_lines(
#         l1["a"], l1["b"], l1["c"],
#         l2["a"], l2["b"], l2["c"]
#     )

#     Pc2 = np.array(data["corner_points"]["Pc2"])
#     Pc3 = np.array(data["corner_points"]["Pc3"])

#     return lines, Pc2, Pc3

# # ================= CBF Functions =================
# def line_cbf(p, line):
#     return line["a"]*p[0] + line["b"]*p[1] + line["c"]

# def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines):
#     xr, yr, th = x

#     if turn == 1:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
#     else:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])

#     if i == 1: return -line_cbf(P1, lines["Lc1"])
#     if i == 2: return -line_cbf(P2, lines["Lc1"])
#     if i == 3: return -line_cbf(P1, lines["Lc2"])
#     if i == 4: return -line_cbf(P4, lines["Lc2"])

#     # --- 2차 함수(포물선) CBF 정의 ---
#     def Cr(p):

#         ex = (P4 - P2)
#         ex = ex / np.linalg.norm(ex)

#         ey = (P1 - P3)
#         ey = ey / np.linalg.norm(ey)

#         def to_local(pt):
#             d = pt - P3
#             xp = np.dot(d, ex)
#             yp = np.dot(d, ey)
#             return xp, yp

#         x2, y2 = to_local(P2)
#         x4, y4 = to_local(P4)

#         eps = 1e-8
#         a2 = y2 / (x2**2 + eps)
#         a4 = y4 / (x4**2 + eps)
#         a = 0.5 * (a2 + a4)

#         xp, yp = to_local(p)
#         return yp - a * xp**2

#     if i == 5: return -Cr(Pc2)
#     if i == 6: return -Cr(Pc3)

# def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines):
#     eps = 1e-4
#     h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines)
#     dhdx = np.zeros(3)

#     for j in range(3):
#         xp = x.copy()
#         xp[j] += eps
#         dhdx[j] = (hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines) - h0) / eps

#     return h0, dhdx

# def plot_footprint(x, L, dl, db, turn):
#     xr, yr, th = x
#     P = np.zeros((2,4))

#     if turn == 1:
#         P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]
#         P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
#         P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
#         P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
#     else:
#         P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
#         P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
#         P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
#         P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]

#     plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]],
#              color='y', alpha=0.15, edgecolor='k')
#     plt.scatter(P[0,:], P[1,:], c='k', s=25)


# # ======== ADD: Cr Plot ========
# def plot_Cr(x, L, dl, db, turn):

#     xr, yr, th = x

#     if turn == 1:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])

#     ex = (P4 - P2)
#     ex /= np.linalg.norm(ex)
#     ey = (P1 - P3)
#     ey /= np.linalg.norm(ey)

#     def to_global(xp, yp):
#         return P3 + xp*ex + yp*ey

#     x2 = np.dot(P2-P3, ex)
#     y2 = np.dot(P2-P3, ey)
#     x4 = np.dot(P4-P3, ex)
#     y4 = np.dot(P4-P3, ey)

#     a = 0.5*((y2/(x2**2+1e-8))+(y4/(x4**2+1e-8)))

#     xs = np.linspace(min(x2,x4)*1.2, max(x2,x4)*1.2, 100)
#     ys = a*xs**2

#     curve = np.array([to_global(xp, yp) for xp, yp in zip(xs, ys)])
#     plt.plot(curve[:,0], curve[:,1], 'g--', linewidth=2)
# # ===============================================


# # ================= Main =================
# if __name__ == "__main__":

#     lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

#     turn = 1
#     x = np.array([ 2.0,  0.0, 0.0])
#     G = np.array([ 4.0, -1.5, -1.57])

#     dt, T = 0.05, 10
#     N = int(T/dt)

#     L, dl, db = 0.8, 0.10, 0.37
#     v_max, w_max = 0.6, 2.5
    
#     p1 = 1.5
#     p2 = 1.5
#     p3 = 1.5
    
#     k_cbf = 1.0

#     traj = np.zeros((3, N))
#     u_traj = np.zeros((3, N))

#     for k in range(N):
#         traj[:,k] = x

#         u_nom = -np.array([
#             p1*(x[0]-G[0]),
#             p2*(x[1]-G[1]),
#             p3*(x[2]-G[2])
#         ])

#         u = cp.Variable(3)
#         cost = 0.5*cp.sum_squares(u) - u_nom @ u
#         cons = []

#         for i in range(6):
#             h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines)
#             cons.append(-dhdx @ u <= k_cbf*h)

#         cons += [
#             u[0] <= v_max, u[0] >= -v_max,
#             u[1] <= v_max, u[1] >= -v_max,
#             u[2] <= w_max, u[2] >= -w_max
#         ]

#         cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
#         u_traj[:,k] = u.value
#         x = x + dt*u.value

#     # ================= Plot =================
#     plt.figure(figsize=(8,8))
#     plt.axis("equal")
#     plt.grid()
#     plt.plot(traj[0], traj[1], 'r', linewidth=3)
#     plt.plot(G[0], G[1], 'ro')

#     plt.plot([1, 5],[0.5, 0.5],'b',linewidth=1.5) 
#     plt.plot([5, 5],[0.5, -2.5],'b',linewidth=1.5)
      
#     # Inner
#     plt.plot([1, 3],[-0.5, -0.5],'b',linewidth=1.5)
#     plt.plot([3, 3],[-0.5, -3.8],'b',linewidth=1.5)

#     for k in range(0, N, N//20):
#         plot_footprint(traj[:,k], L, dl, db, turn)
#         # plot_Cr(traj[:,k], L, dl, db, turn)   # ← 추가

#     plt.show()

#     # ================= Plot Control Inputs =================
#     t = np.arange(N) * dt
#     plt.figure(figsize=(10,6))
#     plt.plot(t, u_traj[0], label="u_x")
#     plt.plot(t, u_traj[1], label="u_y")
#     plt.plot(t, u_traj[2], label="u_theta")
#     plt.axhline(v_max, linestyle='--', alpha=0.3)
#     plt.axhline(-v_max, linestyle='--', alpha=0.3)
#     plt.axhline(w_max, linestyle='--', alpha=0.3)
#     plt.axhline(-w_max, linestyle='--', alpha=0.3)
#     plt.grid()
#     plt.legend()
#     plt.xlabel("time [s]")
#     plt.ylabel("control input")
#     plt.title("Control Inputs")
#     plt.show()



















# import numpy as np
# import cvxpy as cp
# import matplotlib.pyplot as plt
# import yaml
# from pathlib import Path

# # ================= YAML Loader =================
# def load_cbf_map():
#     this_file = Path(__file__).resolve()
#     pkg_root  = this_file.parent.parent
#     yaml_path = pkg_root / "params" / "cbf_heading.yaml"

#     with open(yaml_path, "r") as f:
#         data = yaml.safe_load(f)

#     return data

# def make_lines(a1, b1, c1, a2, b2, c2):
#     return {
#         "Lc1": {"a": a1, "b": b1, "c": c1},
#         "Lc2": {"a": a2, "b": b2, "c": c2},
#     }

# def parse_cbf_yaml(data):
#     l1 = data["cbf_lines"]["Lc1"]
#     l2 = data["cbf_lines"]["Lc2"]

#     lines = make_lines(
#         l1["a"], l1["b"], l1["c"],
#         l2["a"], l2["b"], l2["c"]
#     )

#     Pc2 = np.array(data["corner_points"]["Pc2"])
#     Pc3 = np.array(data["corner_points"]["Pc3"])

#     return lines, Pc2, Pc3

# # ================= CBF Functions =================
# def line_cbf(p, line):
#     return line["a"]*p[0] + line["b"]*p[1] + line["c"]

# # ✅ step_idx 추가
# def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx):

#     xr, yr, th = x

#     if turn == 1:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
#     else:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])

#     if i == 1: return -line_cbf(P1, lines["Lc1"])
#     if i == 2: return -line_cbf(P2, lines["Lc1"])
#     if i == 3: return -line_cbf(P1, lines["Lc2"])
#     if i == 4: return -line_cbf(P4, lines["Lc2"])

#     # --- 2차 함수(포물선) CBF 정의 ---
#     def Cr(p):

#         # ✅ 첫 QP step에서만 P4 → Pc2
#         if step_idx == 0:
#             P4_use = Pc2
#         else:
#             P4_use = P4

#         ex = (P4_use - P2)
#         ex = ex / np.linalg.norm(ex)

#         ey = (P1 - P3)
#         ey = ey / np.linalg.norm(ey)

#         def to_local(pt):
#             d = pt - P3
#             xp = np.dot(d, ex)
#             yp = np.dot(d, ey)
#             return xp, yp

#         x2, y2 = to_local(P2)
#         x4, y4 = to_local(P4_use)

#         eps = 1e-8
#         a2 = y2 / (x2**2 + eps)
#         a4 = y4 / (x4**2 + eps)
#         a = 0.5 * (a2 + a4)

#         xp, yp = to_local(p)
#         return yp - a * xp**2

#     if i == 5: return -Cr(Pc2)
#     if i == 6: return -Cr(Pc3)

# # ✅ step_idx 전달
# def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx):
#     eps = 1e-4
#     h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx)
#     dhdx = np.zeros(3)

#     for j in range(3):
#         xp = x.copy()
#         xp[j] += eps
#         dhdx[j] = (
#             hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx)
#             - h0
#         ) / eps

#     return h0, dhdx

# def plot_footprint(x, L, dl, db, turn):
#     xr, yr, th = x
#     P = np.zeros((2,4))

#     if turn == 1:
#         P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]
#         P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
#         P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
#         P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
#     else:
#         P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
#         P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
#         P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                   yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
#         P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                   yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]

#     plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]],
#              color='y', alpha=0.15, edgecolor='k')
#     plt.scatter(P[0,:], P[1,:], c='k', s=25)


# # ======== ADD: Cr Plot ========
# def plot_Cr(x, L, dl, db, turn):

#     xr, yr, th = x

#     if turn == 1:
#         P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
#         P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
#                        yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
#                        yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])

#     ex = (P4 - P2)
#     ex /= np.linalg.norm(ex)
#     ey = (P1 - P3)
#     ey /= np.linalg.norm(ey)

#     def to_global(xp, yp):
#         return P3 + xp*ex + yp*ey

#     x2 = np.dot(P2-P3, ex)
#     y2 = np.dot(P2-P3, ey)
#     x4 = np.dot(P4-P3, ex)
#     y4 = np.dot(P4-P3, ey)

#     a = 0.5*((y2/(x2**2+1e-8))+(y4/(x4**2+1e-8)))

#     xs = np.linspace(min(x2,x4)*1.2, max(x2,x4)*1.2, 100)
#     ys = a*xs**2

#     curve = np.array([to_global(xp, yp) for xp, yp in zip(xs, ys)])
#     plt.plot(curve[:,0], curve[:,1], 'g--', linewidth=2)
# # ===============================================


# # ================= Main =================
# if __name__ == "__main__":

#     lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

#     turn = 1
#     x = np.array([ 2.0,  0.0, 0.0])
#     G = np.array([ 4.0, -1.5, -1.57])

#     dt, T = 0.05, 10
#     N = int(T/dt)

#     L, dl, db = 0.8, 0.10, 0.37
#     v_max, w_max = 0.6, 2.5
    
#     p1 = 1.5
#     p2 = 1.5
#     p3 = 1.5
    
#     k_cbf = 1.0

#     traj = np.zeros((3, N))
#     u_traj = np.zeros((3, N))

#     for k in range(N):
#         traj[:,k] = x

#         u_nom = -np.array([
#             p1*(x[0]-G[0]),
#             p2*(x[1]-G[1]),
#             p3*(x[2]-G[2])
#         ])

#         u = cp.Variable(3)
#         cost = 0.5*cp.sum_squares(u) - u_nom @ u
#         cons = []

#         for i in range(6):
#             h, dhdx = cbf_i(
#                 x, i+1, L, dl, db,
#                 Pc2, Pc3, turn, lines,
#                 k      # ← step index 전달
#             )
#             cons.append(-dhdx @ u <= k_cbf*h)

#         cons += [
#             u[0] <= v_max, u[0] >= -v_max,
#             u[1] <= v_max, u[1] >= -v_max,
#             u[2] <= w_max, u[2] >= -w_max
#         ]

#         cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
#         u_traj[:,k] = u.value
#         x = x + dt*u.value

#     # ================= Plot =================
#     plt.figure(figsize=(8,8))
#     plt.axis("equal")
#     plt.grid()
#     plt.plot(traj[0], traj[1], 'r', linewidth=3)
#     plt.plot(G[0], G[1], 'ro')

#     plt.plot([1, 5],[0.5, 0.5],'b',linewidth=1.5) 
#     plt.plot([5, 5],[0.5, -2.5],'b',linewidth=1.5)
      
#     # Inner
#     plt.plot([1, 3],[-0.5, -0.5],'b',linewidth=1.5)
#     plt.plot([3, 3],[-0.5, -3.8],'b',linewidth=1.5)

#     for k in range(0, N, N//20):
#         plot_footprint(traj[:,k], L, dl, db, turn)
#         plot_Cr(traj[:,k], L, dl, db, turn)   # ← 추가

#     plt.show()

#     # ================= Plot Control Inputs =================
#     t = np.arange(N) * dt
#     plt.figure(figsize=(10,6))
#     plt.plot(t, u_traj[0], label="u_x")
#     plt.plot(t, u_traj[1], label="u_y")
#     plt.plot(t, u_traj[2], label="u_theta")
#     plt.axhline(v_max, linestyle='--', alpha=0.3)
#     plt.axhline(-v_max, linestyle='--', alpha=0.3)
#     plt.axhline(w_max, linestyle='--', alpha=0.3)
#     plt.axhline(-w_max, linestyle='--', alpha=0.3)
#     plt.grid()
#     plt.legend()
#     plt.xlabel("time [s]")
#     plt.ylabel("control input")
#     plt.title("Control Inputs")
#     plt.show()
    
    
    
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import yaml
from pathlib import Path

# ================= YAML Loader =================
def load_cbf_map():
    this_file = Path(__file__).resolve()
    pkg_root  = this_file.parent.parent
    yaml_path = pkg_root / "params" / "cbf_heading.yaml"

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    return data

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

    return lines, Pc2, Pc3

# ================= CBF Functions =================
def line_cbf(p, line):
    return line["a"]*p[0] + line["b"]*p[1] + line["c"]

def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx):
    xr, yr, th = x

    # 로봇 footprint corner 계산
    if turn == 1:
        P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
        P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
    else:
        P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
        P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])

    # ================== Line CBF ==================
    if i == 1: return -line_cbf(P1, lines["Lc1"])
    if i == 2: return -line_cbf(P2, lines["Lc1"])
    if i == 3: return -line_cbf(P1, lines["Lc2"])
    if i == 4: return -line_cbf(P4, lines["Lc2"])

    # ================== Cr CBF ==================
    def Cr(p):
        # 로컬 좌표계 정의
        ex = (P4 - P2)
        ex /= np.linalg.norm(ex)
        ey = (P1 - P3)
        ey /= np.linalg.norm(ey)

        def to_local(pt):
            d = pt - P3
            xp = np.dot(d, ex)
            yp = np.dot(d, ey)
            return xp, yp

        def to_global(xp, yp):
            return P3 + xp*ex + yp*ey

        x2, y2 = to_local(P2)
        x3, y3 = to_local(P3)  # 최솟값 기준
        x4, y4 = to_local(P4)

        # 2차 함수 a*x^2, P3에서 최솟값
        a = 0.5*((y2/(x2**2 + 1e-8)) + (y4/(x4**2 + 1e-8)))

        xp, yp = to_local(p)
        return yp - a*xp**2

    if i == 5: return -Cr(Pc2)
    if i == 6: return -Cr(Pc3)

def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx):
    eps = 1e-4
    h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx)
    dhdx = np.zeros(3)
    for j in range(3):
        xp = x.copy()
        xp[j] += eps
        dhdx[j] = (hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx) - h0)/eps
    return h0, dhdx

def plot_footprint(x, L, dl, db, turn):
    xr, yr, th = x
    P = np.zeros((2,4))

    if turn == 1:
        P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                  yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]
        P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                  yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
        P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                  yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
        P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                  yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
    else:
        P[:,0] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                  yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)]
        P[:,1] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                  yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
        P[:,2] = [xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                  yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
        P[:,3] = [xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                  yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)]

    plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]], color='y', alpha=0.15, edgecolor='k')
    plt.scatter(P[0,:], P[1,:], c='k', s=25)

# ================= Plot Cr Curve =================
def plot_Cr(x, L, dl, db, turn):
    xr, yr, th = x

    if turn == 1:
        P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])
        P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
    else:
        P1 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) + db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) - db*np.cos(th)])
        P2 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P3 = np.array([xr - (L/2)*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                       yr - (L/2)*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P4 = np.array([xr + (L/2)*np.cos(th) + dl*np.cos(th) - db*np.sin(th),
                       yr + (L/2)*np.sin(th) + dl*np.sin(th) + db*np.cos(th)])

    ex = (P4 - P2); ex /= np.linalg.norm(ex)
    ey = (P1 - P3); ey /= np.linalg.norm(ey)

    def to_global(xp, yp):
        return P3 + xp*ex + yp*ey

    x2 = np.dot(P2-P3, ex)
    x4 = np.dot(P4-P3, ex)
    y2 = np.dot(P2-P3, ey)
    y4 = np.dot(P4-P3, ey)

    a = 0.5*((y2/(x2**2 + 1e-8)) + (y4/(x4**2 + 1e-8)))

    xs = np.linspace(min(x2,x4)*1.2, max(x2,x4)*1.2, 100)
    ys = a*xs**2

    curve = np.array([to_global(xp, yp) for xp, yp in zip(xs, ys)])
    plt.plot(curve[:,0], curve[:,1], 'g--', linewidth=2)

# ================= Main =================
if __name__ == "__main__":
    lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

    turn = 1
    x = np.array([ 2.0,  0.0, 0.0])
    G = np.array([ 4.0, -1.5, 0.0])

    dt, T = 0.05, 10
    N = int(T/dt)

    L, dl, db = 0.8, 0.10, 0.37
    v_max, w_max = 0.6, 2.5
    
    p1 = 1.5; p2 = 1.5; p3 = 1.5
    k_cbf = 1.0

    traj = np.zeros((3, N))
    u_traj = np.zeros((3, N))

    for k in range(N):
        traj[:,k] = x
        u_nom = -np.array([p1*(x[0]-G[0]), p2*(x[1]-G[1]), p3*(x[2]-G[2])])

        u = cp.Variable(3)
        cost = 0.5*cp.sum_squares(u) - u_nom @ u
        cons = []

        for i in range(6):
            h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines, k)
            cons.append(-dhdx @ u <= k_cbf*h)

        cons += [
            u[0] <= v_max, u[0] >= -v_max,
            u[1] <= v_max, u[1] >= -v_max,
            u[2] <= w_max, u[2] >= -w_max
        ]

        cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
        u_traj[:,k] = u.value
        x = x + dt*u.value

    # ================= Plot =================
    plt.figure(figsize=(8,8))
    plt.axis("equal")
    plt.grid()
    plt.plot(traj[0], traj[1], 'r', linewidth=3)
    plt.plot(G[0], G[1], 'ro')

    # Walls
    plt.plot([1, 5],[0.5, 0.5],'b',linewidth=1.5) 
    plt.plot([5, 5],[0.5, -2.5],'b',linewidth=1.5)
    plt.plot([1, 3],[-0.5, -0.5],'b',linewidth=1.5)
    plt.plot([3, 3],[-0.5, -3.8],'b',linewidth=1.5)

    for k in range(0, N, N//20):
        plot_footprint(traj[:,k], L, dl, db, turn)
        plot_Cr(traj[:,k], L, dl, db, turn)

    plt.show()

    # ================= Plot Control Inputs =================
    t = np.arange(N) * dt
    plt.figure(figsize=(10,6))
    plt.plot(t, u_traj[0], label="u_x")
    plt.plot(t, u_traj[1], label="u_y")
    plt.plot(t, u_traj[2], label="u_theta")
    plt.axhline(v_max, linestyle='--', alpha=0.3)
    plt.axhline(-v_max, linestyle='--', alpha=0.3)
    plt.axhline(w_max, linestyle='--', alpha=0.3)
    plt.axhline(-w_max, linestyle='--', alpha=0.3)
    plt.grid()
    plt.legend()
    plt.xlabel("time [s]")
    plt.ylabel("control input")
    plt.title("Control Inputs")
    plt.show()