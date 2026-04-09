import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import yaml
from pathlib import Path


# ==========================================================
# YAML LOADER
# ==========================================================
def make_lines(a1,b1,c1,a2,b2,c2):
    return {
        "Lc1":{"a":a1,"b":b1,"c":c1},
        "Lc2":{"a":a2,"b":b2,"c":c2},
    }


def load_maze_yaml():

    this_file = Path(__file__).resolve()
    pkg_root  = this_file.parent.parent
    yaml_path = pkg_root/"params"/"cbf_maze_3to1.yaml"

    with open(yaml_path,"r") as f:
        data = yaml.safe_load(f)

    if isinstance(data,dict):
        data=data["goals"]

    corners=[]

    for c in data:

        l1=c["cbf_lines"]["Lc1"]
        l2=c["cbf_lines"]["Lc2"]

        corners.append({
            "start":np.array(c["start"]),
            "goal":np.array(c["goal"]),
            "lines":make_lines(
                l1["a"],l1["b"],l1["c"],
                l2["a"],l2["b"],l2["c"]),
            "Pc2":np.array(c["corner_points"]["Pc2"]),
            "Pc3":np.array(c["corner_points"]["Pc3"]),
            "turn":c["turn_dir"]
        })

    return corners


# ==========================================================
# VEHICLE CORNERS (⭐ 핵심 통합)
# ==========================================================
def get_vehicle_corners(x, L, dl, db, turn):

    xr, yr, th = x

    if turn == 1:   # LEFT TURN
        P1 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th),
                       yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])

        P2 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th),
                       yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])

        P3 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th),
                       yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])

        P4 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th),
                       yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])

    else:           # RIGHT TURN (swap lateral sign)
        P1 = np.array([xr + (L/2+dl)*np.cos(th) + db*np.sin(th),
                       yr + (L/2+dl)*np.sin(th) - db*np.cos(th)])

        P2 = np.array([xr - (L/2+dl)*np.cos(th) + db*np.sin(th),
                       yr - (L/2+dl)*np.sin(th) - db*np.cos(th)])

        P3 = np.array([xr - (L/2+dl)*np.cos(th) - db*np.sin(th),
                       yr - (L/2+dl)*np.sin(th) + db*np.cos(th)])

        P4 = np.array([xr + (L/2+dl)*np.cos(th) - db*np.sin(th),
                       yr + (L/2+dl)*np.sin(th) + db*np.cos(th)])

    return P1,P2,P3,P4


# ==========================================================
# CBF
# ==========================================================
def line_cbf(p,line):
    return line["a"]*p[0]+line["b"]*p[1]+line["c"]


def get_ar_params(P2,P3,P4,L,dl,db,offset):

    ey=(P2-P3); ey/=np.linalg.norm(ey)
    ex=(P4-P3); ex/=np.linalg.norm(ex)

    sx=L+2*dl
    sy=2*db
    a=2**(1.0/offset)

    return ex,ey,P3,sx,sy,a


def hi(x,i,L,dl,db,Pc2,Pc3,turn,lines,offset):

    P1,P2,P3,P4 = get_vehicle_corners(x,L,dl,db,turn)

    if i<=4:
        if i==1:return -line_cbf(P1,lines["Lc1"])
        if i==2:return -line_cbf(P2,lines["Lc1"])
        if i==3:return -line_cbf(P1,lines["Lc2"])
        if i==4:return -line_cbf(P4,lines["Lc2"])

    ex,ey,origin,sx,sy,a = get_ar_params(
        P2,P3,P4,L,dl,db,offset)

    def Ar_val(p):
        rel=p-origin
        xn=np.dot(rel,ex)/sx
        yn=np.dot(rel,ey)/sy
        return turn*(yn-a**(-xn))

    if i==5:return -Ar_val(Pc2)
    if i==6:return -Ar_val(Pc3)


def cbf_i(x,i,L,dl,db,Pc2,Pc3,turn,lines,offset):

    eps=1e-4
    h0=hi(x,i,L,dl,db,Pc2,Pc3,turn,lines,offset)

    dhdx=np.zeros(3)

    for j in range(3):
        xp=x.copy()
        xp[j]+=eps
        dhdx[j]=(hi(xp,i,L,dl,db,Pc2,Pc3,turn,lines,offset)-h0)/eps

    return h0,dhdx


# ==========================================================
# UTILS
# ==========================================================
def angle_wrap(a):
    return np.arctan2(np.sin(a),np.cos(a))


# ==========================================================
# VISUALIZATION
# ==========================================================
def plot_footprint(x,L,dl,db,turn):

    P1,P2,P3,P4 = get_vehicle_corners(x,L,dl,db,turn)

    P=np.array([P1,P2,P3,P4]).T

    plt.fill(P[0,[0,1,2,3,0]],
             P[1,[0,1,2,3,0]],
             color='y',alpha=0.15,edgecolor='k')


def plot_Ar(x,L,dl,db,offset,turn):

    P1,P2,P3,P4 = get_vehicle_corners(x,L,dl,db,turn)

    ex,ey,origin,sx,sy,a = \
        get_ar_params(P2,P3,P4,L,dl,db,offset)

    xs=np.linspace(0,1.2,120)

    pts=[]
    for xn in xs:
        yn=a**(-xn)
        p=origin + xn*sx*ex + yn*sy*ey
        pts.append(p)

    pts=np.array(pts)
    plt.plot(pts[:,0],pts[:,1],'g--',linewidth=2)


# ==========================================================
# MAIN
# ==========================================================
if __name__=="__main__":

    corners=load_maze_yaml()

    dt=0.05
    N=400

    L,dl,db=3.0,0.10,0.4
    offset=0.01

    v_max,w_max=0.6,2.5

    p1,p2,p3=1.5,1.5,30.0
    k_cbf=1.0

    goal_tol_pos=0.1
    goal_tol_ang=0.1

    traj_all=[]
    x=corners[0]["start"].copy()

    # ================= corner loop =================
    for cid,corner in enumerate(corners):

        print(f"=== Corner {cid+1} ===")

        G=corner["goal"]
        lines=corner["lines"]
        Pc2=corner["Pc2"]
        Pc3=corner["Pc3"]
        turn=corner["turn"]

        traj=[]

        for k in range(N):

            traj.append(x.copy())

            ang_err=angle_wrap(x[2]-G[2])

            u_nom=-np.array([
                p1*(x[0]-G[0]),
                p2*(x[1]-G[1]),
                p3*ang_err
            ])

            u=cp.Variable(3)

            cost=0.5*cp.sum_squares(u)-u_nom@u
            cons=[]

            for i in range(6):
                h,dhdx=cbf_i(
                    x,i+1,L,dl,db,
                    Pc2,Pc3,turn,lines,offset
                )
                cons.append(-dhdx@u<=k_cbf*h)

            cons+= [
                u[0]<=v_max,u[0]>=-v_max,
                u[1]<=v_max,u[1]>=-v_max,
                u[2]<=w_max,u[2]>=-w_max
            ]

            prob=cp.Problem(cp.Minimize(cost),cons)
            prob.solve(solver=cp.OSQP,warm_start=True)

            if u.value is None:
                print("QP infeasible")
                break

            x=x+dt*u.value

            ang_err=angle_wrap(x[2]-G[2])

            if (np.linalg.norm(x[:2]-G[:2])<goal_tol_pos
                and abs(ang_err)<goal_tol_ang):
                print("Goal reached")
                break

        traj_all.append(np.array(traj).T)

    # ================= plot =================
    plt.figure(figsize=(8,8))
    plt.axis("equal")
    plt.grid()

    for traj in traj_all:
        plt.plot(traj[0], traj[1], 'r', linewidth=3)

    for c in corners:
        plt.plot(c["goal"][0], c["goal"][1], 'ro')

    plt.plot([-2,8],[1,1],'b')
    plt.plot([8,8],[1,-4],'b')
    plt.plot([-2,4],[-1,-1],'b')
    plt.plot([4,4],[-1,-6],'b')
    plt.plot([8,12],[-4,-4],'b')
    plt.plot([4,12],[-6,-6],'b')

    for traj,corner in zip(traj_all,corners):

        turn=corner["turn"]
        N_local=traj.shape[1]

        for k in range(0,N_local,max(1,N_local//20)):
            plot_footprint(traj[:,k],L,dl,db,turn)
            plot_Ar(traj[:,k],L,dl,db,offset,turn)

    plt.title("Sequential Corner CBF Navigation")
    plt.show()
