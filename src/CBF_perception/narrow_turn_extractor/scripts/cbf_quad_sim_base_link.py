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

    plt.fill(P[0,[0,1,2,3,0]], P[1,[0,1,2,3,0]],
             color='y', alpha=0.15, edgecolor='k')
    plt.scatter(P[0,:], P[1,:], c='k', s=25)

# ================= Main =================
if __name__ == "__main__":

    lines, Pc2, Pc3 = parse_cbf_yaml(load_cbf_map())

    turn = 1
    x = np.array([-4.0, -2.0, np.pi/2])
    G = np.array([-1.75, 3.0, 0.0])

    dt, T = 0.05, 80
    N = int(T/dt)

    L, dl, db = 3.0, 0.25, 0.35
    v_max, w_max = 2.5, 2.5     # 0.2 0.25
    
    p1 = 0.9
    p2 = 0.9
    p3 = 0.9
    
    k_cbf = 0.1

    traj = np.zeros((3, N))
    u_traj = np.zeros((3, N))

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

        for i in range(6):
            h, dhdx = cbf_i(x, i+1, L, dl, db, Pc2, Pc3, turn, lines)
            cons.append(-dhdx @ u <= k_cbf*h)

        cons += [
            u[0] <= v_max, u[0] >= -v_max,
            u[1] <= v_max, u[1] >= -v_max,
            u[2] <= w_max, u[2] >= -w_max
        ]

        cp.Problem(cp.Minimize(cost), cons).solve(solver=cp.OSQP)
        u_traj[:,k] = u.value
        x = x + dt*u.value

    # ================= Plot Trajectory =================
    plt.figure(figsize=(8,8))
    plt.axis("equal")
    plt.grid()
    plt.plot(traj[0], traj[1], 'r', linewidth=3)
    plt.plot(G[0], G[1], 'ro')

    for k in range(0, N, N//20):
        plot_footprint(traj[:,k], L, dl, db, turn)
        
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

    plt.title("Trajectory & Footprints")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
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
