# import numpy as np
# import cvxpy as cp
# import matplotlib.pyplot as plt
# import yaml
# from pathlib import Path

# # ================= YAML Loader =================
# def load_cbf_map():
#     this_file = Path(__file__).resolve()
#     pkg_root  = this_file.parent.parent
#     yaml_path = pkg_root / "params" / "cbf_heading_3to1.yaml"
#     with open(yaml_path, "r") as f:
#         data = yaml.safe_load(f)
#     return data

# def make_lines(a1, b1, c1, a2, b2, c2):
#     return {"Lc1": {"a": a1, "b": b1, "c": c1}, "Lc2": {"a": a2, "b": b2, "c": c2}}

# def parse_cbf_yaml(data):
#     l1 = data["cbf_lines"]["Lc1"]; l2 = data["cbf_lines"]["Lc2"]
#     lines = make_lines(l1["a"], l1["b"], l1["c"], l2["a"], l2["b"], l2["c"])
#     Pc2 = np.array(data["corner_points"]["Pc2"])
#     Pc3 = np.array(data["corner_points"]["Pc3"])
#     return lines, Pc2, Pc3

# # ================= New CBF Helper Functions =================
# def get_asymmetric_params(P2, P3, P4):
#     """
#     P3를 꼭짓점(Vertex, 최솟값)으로 하고 P2, P4를 지나는 비대칭 포물선 파라미터 계산
#     """
#     v2 = P2 - P3
#     v4 = P4 - P3
    
#     # 1. 로컬 y축: 두 벡터의 각 이등분선 (P3에서 로봇 안쪽 방향)
#     u2 = v2 / np.linalg.norm(v2)
#     u4 = v4 / np.linalg.norm(v4)
#     ey = (u2 + u4)
#     ey /= np.linalg.norm(ey)
    
#     # 2. 로컬 x축: ey에 수직 (P2가 있는 쪽을 양의 방향으로)
#     ex = np.array([-ey[1], ey[0]])
#     if np.dot(v2, ex) < 0: ex = -ex
    
#     # 3. P2, P4를 로컬 좌표로 투영
#     x2L, y2L = np.dot(v2, ex), np.dot(v2, ey)
#     x4L, y4L = np.dot(v4, ex), np.dot(v4, ey)
    
#     # 4. 각 영역별 계수 a (y = a*x^2)
#     a_pos = y2L / (x2L**2 + 1e-8) # x > 0 영역 (P2 방향)
#     a_neg = y4L / (x4L**2 + 1e-8) # x < 0 영역 (P4 방향)
    
#     return ex, ey, P3, a_pos, a_neg

# # ================= CBF Functions =================
# def line_cbf(p, line):
#     return line["a"]*p[0] + line["b"]*p[1] + line["c"]

# def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx):
#     xr, yr, th = x
#     # Corner Points (Robot Footprint)
#     if turn == 1:
#         P1 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])
#     else:
#         P1 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P2 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P3 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P4 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])

#     # ================== Line CBF (i=1~4) ==================
#     if i <= 4:
#         if i == 1: return -line_cbf(P1, lines["Lc1"])
#         if i == 2: return -line_cbf(P2, lines["Lc1"])
#         if i == 3: return -line_cbf(P1, lines["Lc2"])
#         if i == 4: return -line_cbf(P4, lines["Lc2"])

#     # ================== Asymmetric Quadratic Cr CBF (i=5,6) ==================
#     ex, ey, origin, a_pos, a_neg = get_asymmetric_params(P2, P3, P4)
    
#     def Cr_val(p_target):
#         rel_p = p_target - origin
#         xl = np.dot(rel_p, ex)
#         yl = np.dot(rel_p, ey)
        
#         # 꼭짓점 P3를 기준으로 영역을 나누어 계수 적용
#         a = a_pos if xl >= 0 else a_neg
#         h = yl - a * (xl**2)
#         return turn * h

#     if i == 5: return -Cr_val(Pc2)
#     if i == 6: return -Cr_val(Pc3)

# def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx):
#     eps = 1e-4
#     h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx)
#     dhdx = np.zeros(3)
#     for j in range(3):
#         xp = x.copy(); xp[j] += eps
#         dhdx[j] = (hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx) - h0)/eps
#     return h0, dhdx

# # ================= Visualization =================
# def plot_footprint(x, L, dl, db, turn):
#     xr, yr, th = x
#     P = np.zeros((2,4))
#     if turn == 1:
#         P[:,0] = [xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)]
#         P[:,1] = [xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)]
#         P[:,2] = [xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)]
#         P[:,3] = [xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)]
#     else:
#         P[:,0] = [xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)]
#         P[:,1] = [xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)]
#         P[:,2] = [xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)]
#         P[:,3] = [xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)]
#     plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]], color='y', alpha=0.15, edgecolor='k')
#     plt.scatter(P[0,:], P[1,:], c='k', s=25)

# def plot_Cr(x, L, dl, db, turn):
#     xr, yr, th = x
#     if turn == 1:
#         P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])
#     else:
#         P2 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P3 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P4 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])

#     ex, ey, origin, a_pos, a_neg = get_asymmetric_params(P2, P3, P4)
    
#     # 로컬 x 범위 계산
#     x2L = np.dot(P2 - origin, ex)
#     x4L = np.dot(P4 - origin, ex)
    
#     # 각 영역별 궤적 생성
#     xl_vals = np.linspace(x4L, x2L, 100)
#     pts = []
#     for xl in xl_vals:
#         a = a_pos if xl >= 0 else a_neg
#         yl = a * (xl**2)
#         pts.append(origin + xl*ex + yl*ey)
    
#     pts = np.array(pts)
#     plt.plot(pts[:,0], pts[:,1], 'g--', linewidth=2, alpha=0.8)

# # ================= Main =================
# if __name__ == "__main__":
#     lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

#     turn = 1
#     x = np.array([ 2.5,  0.0, 0.0])
#     G = np.array([ 6.0, -3.0, 0.0])

#     dt, T = 0.05, 10
#     N = int(T/dt)

#     L, dl, db = 2.8, 0.10, 0.35
#     v_max, w_max = 0.6, 2.5
    
#     p1 = 1.5; p2 = 1.5; p3 = 30.0
#     k_cbf = 1.0

#     traj = np.zeros((3, N))
#     u_traj = np.zeros((3, N))

#     for k in range(N):
#         traj[:,k] = x
#         u_nom = -np.array([p1*(x[0]-G[0]), p2*(x[1]-G[1]), p3*(x[2]-G[2])])

#         u = cp.Variable(3)
#         cost = 0.5*cp.sum_squares(u) - u_nom @ u
#         cons = []

#         for i in range(6):
#             h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines, k)
#             cons.append(-dhdx @ u <= k_cbf*h)

#         cons += [
#             u[0] <= v_max, u[0] >= -v_max,
#             u[1] <= v_max, u[1] >= -v_max,
#             u[2] <= w_max, u[2] >= -w_max
#         ]

#         cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
#         if u.value is not None:
#             u_traj[:,k] = u.value
#             x = x + dt*u.value
#         else:
#             u_traj[:,k] = 0.0

#     # ================= Plot =================
#     plt.figure(figsize=(8,8))
#     plt.axis("equal")
#     plt.grid()
#     plt.plot(traj[0], traj[1], 'r', linewidth=3)
#     plt.plot(G[0], G[1], 'ro')

#     # Walls (Basic lines)
#     plt.plot([-2.0, 8.0],[ 1.0,  1.0],'b',linewidth=1.5) 
#     plt.plot([ 8.0, 8.0],[ 1.0, -6.0],'b',linewidth=1.5)

      
#     # Inner
#     plt.plot([-2.0, 4.0],[-1.0, -1.0],'b',linewidth=1.5)
#     plt.plot([4.0, 4.0],[-1.0, -6.0],'b',linewidth=1.5)


#     for k in range(0, N, N//20):
#         plot_footprint(traj[:,k], L, dl, db, turn)
#         plot_Cr(traj[:,k], L, dl, db, turn)

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
#     return {"Lc1": {"a": a1, "b": b1, "c": c1}, "Lc2": {"a": a2, "b": b2, "c": c2}}

# def parse_cbf_yaml(data):
#     l1 = data["cbf_lines"]["Lc1"]; l2 = data["cbf_lines"]["Lc2"]
#     lines = make_lines(l1["a"], l1["b"], l1["c"], l2["a"], l2["b"], l2["c"])
#     Pc2 = np.array(data["corner_points"]["Pc2"])
#     Pc3 = np.array(data["corner_points"]["Pc3"])
#     return lines, Pc2, Pc3

# # ================= New CBF Helper Functions =================
# def get_asymmetric_params(P2, P3, P4):
#     """
#     P3를 꼭짓점(Vertex, 최솟값)으로 하고 P2, P4를 지나는 비대칭 포물선 파라미터 계산
#     """
#     v2 = P2 - P3
#     v4 = P4 - P3
    
#     # 1. 로컬 y축: 두 벡터의 각 이등분선 (P3에서 로봇 안쪽 방향)
#     u2 = v2 / np.linalg.norm(v2)
#     u4 = v4 / np.linalg.norm(v4)
#     ey = (u2 + u4)
#     ey /= np.linalg.norm(ey)
    
#     # 2. 로컬 x축: ey에 수직 (P2가 있는 쪽을 양의 방향으로)
#     ex = np.array([-ey[1], ey[0]])
#     if np.dot(v2, ex) < 0: ex = -ex
    
#     # 3. P2, P4를 로컬 좌표로 투영
#     x2L, y2L = np.dot(v2, ex), np.dot(v2, ey)
#     x4L, y4L = np.dot(v4, ex), np.dot(v4, ey)
    
#     # 4. 각 영역별 계수 a (y = a*x^2)
#     a_pos = y2L / (x2L**2 + 1e-8) # x > 0 영역 (P2 방향)
#     a_neg = y4L / (x4L**2 + 1e-8) # x < 0 영역 (P4 방향)
    
#     return ex, ey, P3, a_pos, a_neg

# # ================= CBF Functions =================
# def line_cbf(p, line):
#     return line["a"]*p[0] + line["b"]*p[1] + line["c"]

# def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx):
#     xr, yr, th = x
#     # Corner Points (Robot Footprint)
#     if turn == 1:
#         P1 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])
#     else:
#         P1 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P2 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P3 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P4 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])

#     # ================== Line CBF (i=1~4) ==================
#     if i <= 4:
#         if i == 1: return -line_cbf(P1, lines["Lc1"])
#         if i == 2: return -line_cbf(P2, lines["Lc1"])
#         if i == 3: return -line_cbf(P1, lines["Lc2"])
#         if i == 4: return -line_cbf(P4, lines["Lc2"])

#     # ================== Asymmetric Quadratic Cr CBF (i=5,6) ==================
#     ex, ey, origin, a_pos, a_neg = get_asymmetric_params(P2, P3, P4)
    
#     def Cr_val(p_target):
#         rel_p = p_target - origin
#         xl = np.dot(rel_p, ex)
#         yl = np.dot(rel_p, ey)
        
#         # 꼭짓점 P3를 기준으로 영역을 나누어 계수 적용
#         a = a_pos if xl >= 0 else a_neg
#         h = yl - a * (xl**2)
#         return turn * h

#     if i == 5: return -Cr_val(Pc2)
#     if i == 6: return -Cr_val(Pc3)

# def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx):
#     eps = 1e-4
#     h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx)
#     dhdx = np.zeros(3)
#     for j in range(3):
#         xp = x.copy(); xp[j] += eps
#         dhdx[j] = (hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines, step_idx) - h0)/eps
#     return h0, dhdx

# # ================= Visualization =================
# def plot_footprint(x, L, dl, db, turn):
#     xr, yr, th = x
#     P = np.zeros((2,4))
#     if turn == 1:
#         P[:,0] = [xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)]
#         P[:,1] = [xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)]
#         P[:,2] = [xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)]
#         P[:,3] = [xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)]
#     else:
#         P[:,0] = [xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)]
#         P[:,1] = [xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)]
#         P[:,2] = [xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)]
#         P[:,3] = [xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)]
#     plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]], color='y', alpha=0.15, edgecolor='k')
#     plt.scatter(P[0,:], P[1,:], c='k', s=25)

# def plot_Cr(x, L, dl, db, turn):
#     xr, yr, th = x
#     if turn == 1:
#         P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th), yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])
#     else:
#         P2 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th), yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])
#         P3 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th), yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])
#         P4 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th), yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])

#     ex, ey, origin, a_pos, a_neg = get_asymmetric_params(P2, P3, P4)
    
#     # 로컬 x 범위 계산
#     x2L = np.dot(P2 - origin, ex)
#     x4L = np.dot(P4 - origin, ex)
    
#     # 각 영역별 궤적 생성
#     xl_vals = np.linspace(x4L, x2L, 100)
#     pts = []
#     for xl in xl_vals:
#         a = a_pos if xl >= 0 else a_neg
#         yl = a * (xl**2)
#         pts.append(origin + xl*ex + yl*ey)
    
#     pts = np.array(pts)
#     plt.plot(pts[:,0], pts[:,1], 'g--', linewidth=2, alpha=0.8)

# # ================= Main =================
# if __name__ == "__main__":
#     lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

#     turn = 1
#     x = np.array([ 2.0,  0.0, 0.0])
#     G = np.array([ 4.0, -1.5, 0.0])

#     dt, T = 0.05, 10
#     N = int(T/dt)

#     L, dl, db = 0.8, 0.10, 0.37
#     v_max, w_max = 0.6, 2.5
    
#     p1 = 1.5; p2 = 1.5; p3 = 1.5
#     k_cbf = 1.0

#     traj = np.zeros((3, N))
#     u_traj = np.zeros((3, N))

#     for k in range(N):
#         traj[:,k] = x
#         u_nom = -np.array([p1*(x[0]-G[0]), p2*(x[1]-G[1]), p3*(x[2]-G[2])])

#         u = cp.Variable(3)
#         cost = 0.5*cp.sum_squares(u) - u_nom @ u
#         cons = []

#         for i in range(6):
#             h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines, k)
#             cons.append(-dhdx @ u <= k_cbf*h)

#         cons += [
#             u[0] <= v_max, u[0] >= -v_max,
#             u[1] <= v_max, u[1] >= -v_max,
#             u[2] <= w_max, u[2] >= -w_max
#         ]

#         cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
#         if u.value is not None:
#             u_traj[:,k] = u.value
#             x = x + dt*u.value
#         else:
#             u_traj[:,k] = 0.0

#     # ================= Plot =================
#     plt.figure(figsize=(8,8))
#     plt.axis("equal")
#     plt.grid()
#     plt.plot(traj[0], traj[1], 'r', linewidth=3)
#     plt.plot(G[0], G[1], 'ro')

#     # Walls (Basic lines)
#     plt.plot([1, 5],[0.5, 0.5],'b',linewidth=1.5) 
#     plt.plot([5, 5],[0.5, -2.5],'b',linewidth=1.5)
#     plt.plot([1, 3],[-0.5, -0.5],'b',linewidth=1.5)
#     plt.plot([3, 3],[-0.5, -3.8],'b',linewidth=1.5)


#     for k in range(0, N, N//20):
#         plot_footprint(traj[:,k], L, dl, db, turn)
#         plot_Cr(traj[:,k], L, dl, db, turn)

#     plt.show()




# =========== y=a^(-x)
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import yaml
from pathlib import Path

# ================= YAML Loader =================
def load_cbf_map():
    this_file = Path(__file__).resolve()
    pkg_root  = this_file.parent.parent
    yaml_path = pkg_root / "params" / "cbf_heading_3to1.yaml"
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    return data


def make_lines(a1, b1, c1, a2, b2, c2):
    return {"Lc1": {"a": a1, "b": b1, "c": c1},
            "Lc2": {"a": a2, "b": b2, "c": c2}}


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


# ==========================================================
# Ar Barrier (Exponential)
# ==========================================================
def get_ar_params(P2, P3, P4, L, dl, db, offset):

    ey = (P2 - P3)
    ey /= np.linalg.norm(ey)

    ex = (P4 - P3)
    ex /= np.linalg.norm(ex)

    sx = (L + 2*dl)
    sy = (2*db)

    a = 2**(1.0/offset)

    return ex, ey, P3, sx, sy, a


# ================= CBF =================
def line_cbf(p, line):
    return line["a"]*p[0] + line["b"]*p[1] + line["c"]


def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, offset):

    xr, yr, th = x

    # -------- footprint --------
    P1 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th),
                   yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])

    P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th),
                   yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])

    P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th),
                   yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])

    P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th),
                   yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])

    # ---------- Line CBF ----------
    if i <= 4:
        if i == 1: return -line_cbf(P1, lines["Lc1"])
        if i == 2: return -line_cbf(P2, lines["Lc1"])
        if i == 3: return -line_cbf(P1, lines["Lc2"])
        if i == 4: return -line_cbf(P4, lines["Lc2"])

    # ---------- Ar exponential ----------
    ex, ey, origin, sx, sy, a = \
        get_ar_params(P2, P3, P4, L, dl, db, offset)

    def Ar_val(p):
        rel = p - origin

        xl = np.dot(rel, ex)
        yl = np.dot(rel, ey)

        xn = xl / sx
        yn = yl / sy

        h = yn - a**(-xn)

        return turn*h

    if i == 5: return -Ar_val(Pc2)
    if i == 6: return -Ar_val(Pc3)


def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines, offset):

    eps = 1e-4
    h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, offset)

    dhdx = np.zeros(3)

    for j in range(3):
        xp = x.copy()
        xp[j] += eps
        dhdx[j] = (
            hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines, offset) - h0
        ) / eps

    return h0, dhdx


# ==========================================================
# Visualization
# ==========================================================
def plot_footprint(x, L, dl, db):

    xr, yr, th = x

    P = np.zeros((2,4))

    P[:,0] = [xr + (L/2+dl)*np.cos(th) - db*np.sin(th),
              yr + (L/2+dl)*np.sin(th) + db*np.cos(th)]

    P[:,1] = [xr - (L/2+dl)*np.cos(th) - db*np.sin(th),
              yr - (L/2+dl)*np.sin(th) + db*np.cos(th)]

    P[:,2] = [xr - (L/2+dl)*np.cos(th) + db*np.sin(th),
              yr - (L/2+dl)*np.sin(th) - db*np.cos(th)]

    P[:,3] = [xr + (L/2+dl)*np.cos(th) + db*np.sin(th),
              yr + (L/2+dl)*np.sin(th) - db*np.cos(th)]

    plt.fill(P[0,[0,1,2,3,0]],
             P[1,[0,1,2,3,0]],
             color='y', alpha=0.15, edgecolor='k')


def plot_Ar(x, L, dl, db, offset):

    xr, yr, th = x

    P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th),
                   yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])

    P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th),
                   yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])

    P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th),
                   yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])

    ex, ey, origin, sx, sy, a = \
        get_ar_params(P2, P3, P4, L, dl, db, offset)

    xs = np.linspace(0, 1.2, 120)

    pts = []
    for xn in xs:
        yn = a**(-xn)
        p = origin + xn*sx*ex + yn*sy*ey
        pts.append(p)

    pts = np.array(pts)
    plt.plot(pts[:,0], pts[:,1], 'g--', linewidth=2)


# ==========================================================
# Main
# ==========================================================
if __name__ == "__main__":

    lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

    x = np.array([2.5, 0.0, 0.0])
    G = np.array([6.0,-3.0, 1.57])

    dt = 0.05
    N = 200

    L, dl, db = 3.0, 0.10, 0.4
    offset = 0.01

    # ⭐ velocity limits
    v_max, w_max = 0.6, 2.5

    traj = np.zeros((3,N))

    for k in range(N):

        traj[:,k] = x

        u_nom = -np.array([
            1.5*(x[0]-G[0]),
            1.5*(x[1]-G[1]),
            30.0*(x[2]-G[2])
        ])

        u = cp.Variable(3)

        cost = 0.5*cp.sum_squares(u) - u_nom @ u
        cons = []

        # ---------- CBF constraints ----------
        for i in range(6):
            h, dhdx = cbf_i(
                x, i+1, L, dl, db,
                Pc2, Pc3, 1, lines, offset
            )
            cons.append(-dhdx @ u <= h)

        # ---------- velocity bounds ----------
        cons += [
            u[0] <= v_max,
            u[0] >= -v_max,
            u[1] <= v_max,
            u[1] >= -v_max,
            u[2] <= w_max,
            u[2] >= -w_max,
        ]

        cp.Problem(cp.Minimize(cost), cons).solve(
            solver=cp.OSQP
        )

        if u.value is not None:
            x = x + dt * u.value

    # ================= Plot =================
    plt.figure(figsize=(8,8))
    plt.axis("equal")
    plt.grid()

    plt.plot(traj[0], traj[1], 'r', linewidth=3)
    plt.plot(G[0], G[1], 'ro')

    # walls
    plt.plot([-2,8],[1,1],'b')
    plt.plot([8,8],[1,-6],'b')
    plt.plot([-2,4],[-1,-1],'b')
    plt.plot([4,4],[-1,-6],'b')

    for k in range(0, N, N//20):
        plot_footprint(traj[:,k], L, dl, db)
        plot_Ar(traj[:,k], L, dl, db, offset)

    plt.show()


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
#     return {"Lc1": {"a": a1, "b": b1, "c": c1},
#             "Lc2": {"a": a2, "b": b2, "c": c2}}


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


# # ==========================================================
# # Ar Barrier (Exponential)
# # ==========================================================
# def get_ar_params(P2, P3, P4, L, dl, db, offset):

#     ey = (P2 - P3)
#     ey /= np.linalg.norm(ey)

#     ex = (P4 - P3)
#     ex /= np.linalg.norm(ex)

#     sx = (L + 2*dl)
#     sy = (2*db)

#     a = 2**(1.0/offset)

#     return ex, ey, P3, sx, sy, a


# # ================= CBF =================
# def line_cbf(p, line):
#     return line["a"]*p[0] + line["b"]*p[1] + line["c"]


# def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, offset):

#     xr, yr, th = x

#     # -------- footprint --------
#     P1 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th),
#                    yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])

#     P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th),
#                    yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])

#     P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th),
#                    yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])

#     P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th),
#                    yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])

#     # ---------- Line CBF ----------
#     if i <= 4:
#         if i == 1: return -line_cbf(P1, lines["Lc1"])
#         if i == 2: return -line_cbf(P2, lines["Lc1"])
#         if i == 3: return -line_cbf(P1, lines["Lc2"])
#         if i == 4: return -line_cbf(P4, lines["Lc2"])

#     # ---------- Ar exponential ----------
#     ex, ey, origin, sx, sy, a = \
#         get_ar_params(P2, P3, P4, L, dl, db, offset)

#     def Ar_val(p):
#         rel = p - origin

#         xl = np.dot(rel, ex)
#         yl = np.dot(rel, ey)

#         xn = xl / sx
#         yn = yl / sy

#         h = yn - a**(-xn)

#         return turn*h

#     if i == 5: return -Ar_val(Pc2)
#     if i == 6: return -Ar_val(Pc3)


# def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines, offset):

#     eps = 1e-4
#     h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines, offset)

#     dhdx = np.zeros(3)

#     for j in range(3):
#         xp = x.copy()
#         xp[j] += eps
#         dhdx[j] = (
#             hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines, offset) - h0
#         ) / eps

#     return h0, dhdx


# # ==========================================================
# # Visualization
# # ==========================================================
# def plot_footprint(x, L, dl, db):

#     xr, yr, th = x

#     P = np.zeros((2,4))

#     P[:,0] = [xr + (L/2+dl)*np.cos(th) - db*np.sin(th),
#               yr + (L/2+dl)*np.sin(th) + db*np.cos(th)]

#     P[:,1] = [xr - (L/2+dl)*np.cos(th) - db*np.sin(th),
#               yr - (L/2+dl)*np.sin(th) + db*np.cos(th)]

#     P[:,2] = [xr - (L/2+dl)*np.cos(th) + db*np.sin(th),
#               yr - (L/2+dl)*np.sin(th) - db*np.cos(th)]

#     P[:,3] = [xr + (L/2+dl)*np.cos(th) + db*np.sin(th),
#               yr + (L/2+dl)*np.sin(th) - db*np.cos(th)]

#     plt.fill(P[0,[0,1,2,3,0]],
#              P[1,[0,1,2,3,0]],
#              color='y', alpha=0.15, edgecolor='k')


# def plot_Ar(x, L, dl, db, offset):

#     xr, yr, th = x

#     P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th),
#                    yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])

#     P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th),
#                    yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])

#     P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th),
#                    yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])

#     ex, ey, origin, sx, sy, a = \
#         get_ar_params(P2, P3, P4, L, dl, db, offset)

#     xs = np.linspace(0, 1.2, 120)

#     pts = []
#     for xn in xs:
#         yn = a**(-xn)
#         p = origin + xn*sx*ex + yn*sy*ey
#         pts.append(p)

#     pts = np.array(pts)
#     plt.plot(pts[:,0], pts[:,1], 'g--', linewidth=2)


# # ==========================================================
# # Main
# # ==========================================================
# if __name__ == "__main__":

#     lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

#     x = np.array([ 2.0,  0.0, 0.0])
#     G = np.array([ 4.0, -1.5, 0.0])

#     dt = 0.05
#     N = 200

#     L, dl, db = 0.8, 0.10, 0.37
#     v_max, w_max = 0.6, 2.5
    
#     offset = 0.05


#     traj = np.zeros((3,N))

#     for k in range(N):

#         traj[:,k] = x

#         u_nom = -np.array([
#             1.5*(x[0]-G[0]),
#             1.5*(x[1]-G[1]),
#             30.0*(x[2]-G[2])
#         ])

#         u = cp.Variable(3)

#         cost = 0.5*cp.sum_squares(u) - u_nom @ u
#         cons = []

#         # ---------- CBF constraints ----------
#         for i in range(6):
#             h, dhdx = cbf_i(
#                 x, i+1, L, dl, db,
#                 Pc2, Pc3, 1, lines, offset
#             )
#             cons.append(-dhdx @ u <= h)

#         # ---------- velocity bounds ----------
#         cons += [
#             u[0] <= v_max,
#             u[0] >= -v_max,
#             u[1] <= v_max,
#             u[1] >= -v_max,
#             u[2] <= w_max,
#             u[2] >= -w_max,
#         ]

#         cp.Problem(cp.Minimize(cost), cons).solve(
#             solver=cp.OSQP
#         )

#         if u.value is not None:
#             x = x + dt * u.value

#     # ================= Plot =================
#     plt.figure(figsize=(8,8))
#     plt.axis("equal")
#     plt.grid()

#     plt.plot(traj[0], traj[1], 'r', linewidth=3)
#     plt.plot(G[0], G[1], 'ro')

#     # walls
#     plt.plot([1, 5],[0.5, 0.5],'b',linewidth=1.5) 
#     plt.plot([5, 5],[0.5, -2.5],'b',linewidth=1.5)
#     plt.plot([1, 3],[-0.5, -0.5],'b',linewidth=1.5)
#     plt.plot([3, 3],[-0.5, -3.8],'b',linewidth=1.5)

#     for k in range(0, N, N//20):
#         plot_footprint(traj[:,k], L, dl, db)
#         plot_Ar(traj[:,k], L, dl, db, offset)

#     plt.show()