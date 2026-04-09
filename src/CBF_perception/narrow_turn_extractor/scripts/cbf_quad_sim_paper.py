import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import yaml
from pathlib import Path

# ================= YAML Loader =================
def load_cbf_map():
    this_file = Path(__file__).resolve()
    pkg_root  = this_file.parent.parent
    yaml_path = pkg_root / "params" / "cbf_params.yaml"

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

def hi(x, i, L, dl, db, Pc2, Pc3, turn, lines):
    xr, yr, th = x

    if turn == 1:
        P1 = np.array([xr + dl*np.cos(th) - db*np.sin(th),
                       yr + dl*np.sin(th) + db*np.cos(th)])
        P2 = np.array([xr - L*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                       yr - L*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P3 = np.array([xr - L*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                       yr - L*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P4 = np.array([xr + dl*np.cos(th) + db*np.sin(th),
                       yr + dl*np.sin(th) - db*np.cos(th)])
    elif turn == -1:
        P1 = np.array([xr + dl*np.cos(th) + db*np.sin(th),
                       yr + dl*np.sin(th) - db*np.cos(th)])
        P2 = np.array([xr - L*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                       yr - L*np.sin(th) - dl*np.sin(th) - db*np.cos(th)])
        P3 = np.array([xr - L*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                       yr - L*np.sin(th) - dl*np.sin(th) + db*np.cos(th)])
        P4 = np.array([xr + dl*np.cos(th) - db*np.sin(th),
                       yr + dl*np.sin(th) + db*np.cos(th)])

    if i == 1: return -line_cbf(P1, lines["Lc1"])
    if i == 2: return -line_cbf(P2, lines["Lc1"])
    if i == 3: return -line_cbf(P1, lines["Lc2"])
    if i == 4: return -line_cbf(P2, lines["Lc2"])
    
    Lr = lambda p: turn*(
        (P4[0]-P3[0])*(p[1]-P3[1]) -
        (P4[1]-P3[1])*(p[0]-P3[0])
    ) / np.linalg.norm(P4 - P3)

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
        P[:,0] = [xr + dl*np.cos(th) - db*np.sin(th),
                yr + dl*np.sin(th) + db*np.cos(th)]
        P[:,1] = [xr - L*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                yr - L*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
        P[:,2] = [xr - L*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                yr - L*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
        P[:,3] = [xr + dl*np.cos(th) + db*np.sin(th),
                yr + dl*np.sin(th) - db*np.cos(th)]
    elif turn == -1:
        P[:,0] = [xr + dl*np.cos(th) + db*np.sin(th),
                yr + dl*np.sin(th) - db*np.cos(th)]
        P[:,1] = [xr - L*np.cos(th) - dl*np.cos(th) + db*np.sin(th),
                yr - L*np.sin(th) - dl*np.sin(th) - db*np.cos(th)]
        P[:,2] = [xr - L*np.cos(th) - dl*np.cos(th) - db*np.sin(th),
                yr - L*np.sin(th) - dl*np.sin(th) + db*np.cos(th)]
        P[:,3] = [xr + dl*np.cos(th) - db*np.sin(th),
                yr + dl*np.sin(th) + db*np.cos(th)]        

    plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]],
             color='y', alpha=0.15, edgecolor='k')
    plt.scatter(P[0,:], P[1,:], c='k', s=25)
    plt.plot(xr, yr, 'bo', markersize=4)
    plt.plot([P[0,2], P[0,3]], [P[1,2], P[1,3]], 'g--', linewidth=2)

# ================= Main =================
if __name__ == "__main__":

    # Load YAML
    cbf_data = load_cbf_map()
    lines, Pc2, Pc3 = parse_cbf_yaml(cbf_data)

    # Goal1
    turn = 1
    x = np.array([-4.0, -0.5, np.pi/2])
    G = np.array([-0.25, 3.0, np.pi])
    
    # # Goal2
    # turn = -1
    # x = np.array([-0.25, 3.0, 0.0])
    # G = np.array([ 1.0,  7.0, np.pi/2])

    dt = 0.05
    T  = 120
    N  = int(T/dt)

    L, dl, db = 3.0, 0.25, 0.35
    v_max, w_max = 0.2, 0.25
    k_cbf = 0.1
    alpha = lambda h: h
    p1 = 1.5
    p2 = 1.5
    p3 = 30.0

    traj = np.zeros((3, N))

    for k in range(N):
        traj[:,k] = x

        u_nom = -np.array([
            p1*(x[0]-G[0]),
            p2*(x[1]-G[1]),
            p3*(x[2]-G[2])
        ])

        u = cp.Variable(3)
        cost = 0.5*cp.sum_squares(u) - u_nom@u
        cons = []

        for i in range(6):
            h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines)
            cons.append(-dhdx @ u <= k_cbf*h)

        cons += [
            u[0] <= v_max, u[0] >= -v_max,
            u[1] <= v_max, u[1] >= -v_max,
            u[2] <= w_max, u[2] >= -w_max
        ]

        cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
        x = x + dt*u.value

    plt.figure(figsize=(8,8))
    plt.axis('equal')
    plt.grid(True)
    plt.title("Trajectory & Footprints")

    # Trajectory
    plt.plot(traj[0,:], traj[1,:], 'r', linewidth=4)
    plt.plot(G[0], G[1], 'ro', markersize=10)

    # Walls (기존 그대로 유지)
    plt.plot([-5, -5],[-7, 4],'b',linewidth=1.5) 
    plt.plot([-5, 0],[4, 4],'b',linewidth=1.5)  
    plt.plot([0, 0],[4, 12],'b',linewidth=1.5)  
    plt.plot([0, 8],[12, 9],'b',linewidth=1.5) 
    plt.plot([8, 8],[1, 9],'b',linewidth=1.5)   
    plt.plot([8, 4],[1, -1],'b',linewidth=1.5) 
    plt.plot([4, 4],[-1, -7],'b',linewidth=1.5) 
    plt.plot([4, -5],[-7, -7],'b',linewidth=1.5)

    plt.plot([-3, -3],[-5, 2],'b',linewidth=1.5) 
    plt.plot([-3, 2],[2, 2],'b',linewidth=1.5) 
    plt.plot([2, 2],[2, 9],'b',linewidth=1.5) 
    plt.plot([2, 6],[9, 8],'b',linewidth=1.5)  
    plt.plot([6, 6],[2, 8],'b',linewidth=1.5)  
    plt.plot([6, 2],[2, 0],'b',linewidth=1.5)  
    plt.plot([2, 2],[0, -5],'b',linewidth=1.5) 
    plt.plot([2, -3],[-5, -5],'b',linewidth=1.5) 

    # Footprints
    step_fp = N // 20
    for k in range(0, N, step_fp):
        plot_footprint(traj[:, k], L, dl, db, turn)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()
