import roboticstoolbox as rtb
import numpy as np
from spatialmath import *
from spatialmath.base import *
from matplotlib import pyplot as plt
from roboticstoolbox.tools.trajectory import *
from roboticstoolbox.backends.swift import Swift


def check_smooth_traj(sol, jump_threshold=0.15):
    dists = []
    for current_q, next_q in zip(sol.q[1:], sol.q[2:]):
        for j1, j2 in zip(current_q, next_q):
            dist = np.fabs(j1-j2)
            dists.append(dist)
    print(max(dists))
    return max(dists) < jump_threshold


def zadanie_3():
    points_number = 50
    eef_height = 0.15
    r = 0.1
    x0 = 0.65
    y0 = 0.2
    angles = np.linspace(0, 2*np.pi, points_number)

    Pt_list = []
    for angle in angles:
        x = x0 + r*np.cos(angle)
        y = y0 + r*np.sin(angle)
        z = eef_height
        Pt_list.append([x,y,z])

    Pt_list = np.asarray(Pt_list)
    x_toplot = Pt_list[:, 0]
    y_toplot = Pt_list[:, 1]

    #plt.plot(x_toplot, y_toplot)
    #plt.show()

    robot = rtb.models.Panda()

    T_list = robot.fkine(robot.qz)
    T_list.extend(SE3(Pt_list)*SE3.Rx(np.pi))

    smooth_traj = False
    while not smooth_traj:
        sol = robot.ikine_QP(T_list)
        if not sol.success:
            print("IK failed!\n")
            return
        smooth_traj = check_smooth_traj(sol)

    traj = mstraj(sol.q, dt=0.2, tacc=0.2, qdmax=0.3)
    rtb.xplot(traj.q, block=True)

    robot.plot(traj.q, backend="swift", loop=True)


if __name__ == '__main__':
    zadanie_3()
