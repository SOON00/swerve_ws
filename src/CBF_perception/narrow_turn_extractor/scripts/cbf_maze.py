import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import yaml
from pathlib import Path

# ================= YAML Loader =================
def load_cbf_segments():
    this_file = Path(__file__).resolve()
    pkg_root  = this_file.parent.parent
    yaml_path = pkg_root / "params" / "maze_params.yaml"

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

# ================= CBF Functions =================
def line_cbf(p, line):
    return line["a"]*p[0] + line["b"]*p[1] + line["c"]

def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines):
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

    if i == 1: return -line_cbf(P1, lines["Lc1"])
    if i == 2: return -line_cbf(P2, lines["Lc1"])
    if i == 3: return -line_cbf(P1, lines["Lc2"])
    if i == 4: return -line_cbf(P2, lines["Lc2"])

    Lr = lambda p: turn*((P4[0]-P3[0])*(p[1]-P3[1]) -
                          (P4[1]-P3[1])*(p[0]-P3[0])) / np.linalg.norm(P4-P3)

    if i == 5: return -Lr(Pc2)
    if i == 6: return -Lr(Pc3)

def cbf_i(x, i, L, dl, db, Pc2, Pc3, turn, lines):
    eps = 1e-4
    h0 = hi(x, i, L, dl, db, Pc2, Pc3, turn, lines)
    dhdx = np.zeros(3)
    for j in range(3):
        xp = x.copy()
        xp[j] += eps
        dhdx[j] = (hi(xp, i, L, dl, db, Pc2, Pc3, turn, lines) - h0) / eps
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
    # plt.scatter(P[0,:], P[1,:], c='k', s=25)

# ================= Main =================
if __name__ == "__main__":

    segments = load_cbf_segments()

    dt, T_per_seg = 0.05, 80
    L, dl, db = 0.8, 0.10, 0.37
    v_max, w_max = 0.6, 2.5
    p1, p2, p3 = 1.5, 1.5, 1.5
    k_cbf = 1.0
    goal_tol_pos = 0.01
    goal_tol_ang = 1*np.pi/180

    traj_all = []
    u_all = []

    t_log = []
    h_log = []
    u_log = []
    t_now = 0.0

    for seg in segments:
        x = seg["start"].copy()
        G = seg["goal"]
        lines = seg["lines"]
        Pc2, Pc3 = seg["Pc2"], seg["Pc3"]
        turn = seg["turn"]

        N = int(T_per_seg/dt)
        traj = np.zeros((3,N))
        u_traj = np.zeros((3,N))

        for k in range(N):
            traj[:,k] = x

            u_nom = -np.array([
                p1*(x[0]-G[0]),
                p2*(x[1]-G[1]),
                p3*(x[2]-G[2])
            ])

            u = cp.Variable(3)
            cost = 0.5*cp.sum_squares(u) - u_nom @ u
            cons = []

            h_step = np.zeros(6)

            for i in range(6):
                h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines)
                cons.append(-dhdx @ u <= k_cbf*h)
                h_step[i] = h

            cons += [
                u[0] <= v_max, u[0] >= -v_max,
                u[1] <= v_max, u[1] >= -v_max,
                u[2] <= w_max, u[2] >= -w_max
            ]

            cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
            u_traj[:,k] = u.value
            
            u_log.append(u.value)
            h_log.append(h_step)
            t_log.append(t_now)
            
            x = x + dt*u.value
            t_now += dt

            if np.linalg.norm(x[:2]-G[:2]) < goal_tol_pos and abs(np.arctan2(np.sin(x[2]-G[2]), np.cos(x[2]-G[2]))) < goal_tol_ang:
                traj = traj[:,:k+1]
                u_traj = u_traj[:,:k+1]
                break

        traj_all.append(traj)
        u_all.append(u_traj)
        
    t_log = np.array(t_log)
    h_log = np.array(h_log).T   # (6, N)
    u_log = np.array(u_log).T   # (3, N)

    # ================= Plot =================
    plt.figure(figsize=(8,8))
    plt.axis("equal")
    plt.grid()
    
    for traj, seg in zip(traj_all, segments):
        # plt.plot(traj[0], traj[1], 'r', linewidth=3)
        
        # Start: green X
        plt.scatter(seg["start"][0], seg["start"][1],
                    c='red', s=120, marker='x',
                    linewidths=3, zorder=6)

        # Goal: red X
        plt.scatter(seg["goal"][0], seg["goal"][1],
                    c='red', s=120, marker='x',
                    linewidths=3, zorder=6)
        
        for k in range(0, traj.shape[1], max(1,traj.shape[1]//20)):
            plot_footprint(traj[:,k], L, dl, db, seg["turn"])
    
    # Walls
    # Outer
    plt.plot([-1, 4],[ 0.5,  0.5],'b',linewidth=1.5) 
    plt.plot([ 4, 4],[ 0.5, -2.7],'b',linewidth=1.5)
    plt.plot([ 4, 10.7535],[-2.7, -2.7],'b',linewidth=1.5)
    plt.plot([ 4.3035, 10.7535],[-7, -2.7],'b',linewidth=1.5)
    plt.plot([ 2, 4.3035],[-7, -7],'b',linewidth=1.5)
    plt.plot([ 2, 4],[-7, -9],'b',linewidth=1.5)
    plt.plot([ 4, 7],[-9, -9],'b',linewidth=1.5)
    plt.plot([ 7, 7],[-9,-11],'b',linewidth=1.5)
    plt.plot([-1, 7],[-11,-11],'b',linewidth=1.5)
    plt.plot([-3, -1],[-8.0, -11],'b',linewidth=1.5)
    plt.plot([-3, -3],[-2.7, -8],'b',linewidth=1.5)
    plt.plot([-1, -3],[-2.7, -2.7],'b',linewidth=1.5)
    plt.plot([-1, -1],[ 0.5, -2.7],'b',linewidth=1.5)
      
    # Inner
    plt.plot([0.3, 3],[-0.5, -0.5],'b',linewidth=1.5)
    plt.plot([3, 3],[-0.5, -4],'b',linewidth=1.5)
    plt.plot([3, 7],[-4,  -4],'b',linewidth=1.5)
    plt.plot([4, 7],[-6,  -4],'b',linewidth=1.5)
    plt.plot([-0.414, 4],[-6, -6],'b',linewidth=1.5)
    plt.plot([-0.414, 3.586],[-6,  -10],'b',linewidth=1.5)
    plt.plot([-0.104, 6],[ -10,  -10],'b',linewidth=1.5)
    plt.plot([-1.7, -0.104],[-7.606,  -10],'b',linewidth=1.5)
    plt.plot([-1.7, -1.7],[-4,  -7.606],'b',linewidth=1.5)
    plt.plot([-1.7, 0.3],[-4, -4],'b',linewidth=1.5)
    plt.plot([0.3, 0.3],[-4, -0.5],'b',linewidth=1.5)
    
    plt.title("Trajectory & Footprints (Multi-goal)")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()

    # ================= Figure 6a : CBF =================
    plt.figure()
    for i in range(6):
        plt.plot(t_log, h_log[i], linewidth=2)
    plt.axhline(0, linestyle='--')
    plt.xlabel("Time [s]")
    plt.ylabel("CBF h_i")
    plt.title("Evolution of the Control Barrier Functions")
    plt.grid()
    plt.legend([f"h{i+1}" for i in range(6)] + ["h=0"])
    plt.show()

    # ================= Figure 6b : Control =================
    plt.figure()
    plt.plot(t_log, u_log[0], linewidth=2)
    plt.plot(t_log, u_log[1], linewidth=2)
    plt.plot(t_log, u_log[2], linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Control input")
    plt.title("Evolution of the Control Inputs")
    plt.grid()
    plt.legend(["v_x", "v_y", "ω_z"])
    plt.show()