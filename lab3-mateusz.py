# wczytanie potrzebnych podczas zajęć bibliotek:
import numpy as np
from spatialmath import *
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import *
from roboticstoolbox.backends.swift import Swift
import time
# ...

# definicje funkcji:
def przyklad_1():
    traj = quintic(0,1,50) #trajektoria wielomianowa
    #traj.plot(traj)

    traj = quintic(0,1,50,0.5) #niezerowa prękdość początkowa
    #traj.plot(traj)

    traj = trapezoidal(0,1,50,0.025) #trajektoria hybrydowa (trapezoidalna)
    #traj.plot(traj)
    #print(max(traj.qd)) #wartość prędkości na odcinku liniowym

    #Wincyj wymiarów 
    traj = jtraj([0,2],[1,-1],50) #trajektoria wielomianowa dla dwóch zmiennych konfiguracyjnych
    #traj.plot(traj)
    traj = mtraj(quintic, [0,2],[1,-1],50)
    #traj.plot(traj)

    traj = mtraj(trapezoidal, [0,2],[1,-1],50) #trajektoria trapezoidalna dla dwóch zmiennych konfiguracyjnych
    #traj.plot(traj)

    #Trajektoria pomiędzy konfiguracjami robota
    robot = rtb.models.DH.Panda()
    traj = mtraj(quintic, robot.qz, robot.qr, 50)
    #traj.plot(traj)

    #Trajektoria z punktami pośrednimi - via points
    via_pt = np.array([[0,0],[1,0.5],[0.2,2],[0.5,1]]) #punkty wchodzące w skład trajektorii
    traj = mstraj(via_pt, dt=0.02, tacc=0.2, qdmax=2.0)
    #rtb.xplot(traj.q, block=True)

    #Trajektoria w przestrzeni kartezjańskiej
    T1 = SE3(0.4, 0.2, 0) * SE3.RPY([0,0,3])
    T2 = SE3(-0.4, -0.2, 0.3) * SE3.RPY([-np.pi/4, np.pi/4, -np.pi/2])
    cart_craj = ctraj(T1, T2, 50)
    #print(cart_craj)

    #Wizualizacja

    # PyPlot  
    robot = rtb.models.DH.Panda()
    T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
    solution = robot.ikine_LM(T)
    traj = jtraj(robot.qz, solution.q, 50)
    #robot.plot(traj.q, backend = 'pyplot', limits=[-0.25, 1.25, -0.5, 0.5, 0, 1], movie='panda_pyplot.gif')

    # Swift
    robot = rtb.models.Panda()
    T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
    solution = robot.ikine_LM(T)
    traj = jtraj(robot.qz, solution.q, 50)
    #robot.plot(traj.q, backend = 'swift', loop=True)

    #Algorytm RRMT
    # Make and instance of the Swift simulator and open it
    env = Swift()
    env.launch(realtime=True)

    # Make a robot model and set its joint angles to the ready joint configuration
    robot = rtb.models.Panda()
    robot.q = robot.qr

    # Set a desired and effector pose an an offset from the current end-effector pose
    Tep = robot.fkine(robot.q) * SE3.Tx(0.2) * SE3.Ty(0.2) * SE3.Tz(0.45) * SE3.Rx(np.pi/4)

    # Add the robot to the simulator
    env.add(robot)
    time.sleep(2)

    # Simulate the robot while it has not arrived at the goal
    arrived = False
    while not arrived:
        # Work out the required end-effector velocity to go towards the goal
        v, arrived = rtb.p_servo(robot.fkine(robot.q), Tep, 1)

        # Set the robot's joint velocities (calculate pseudoinverse(J) * v)
        robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v

        # Step the simulator by 5 milliseconds
        env.step(0.005)

def zadanie_1():
    robot = rtb.models.DH.Panda()
    T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
    solution = robot.ikine_LM(T)
    traj = jtraj(robot.qz, solution.q, 50)
    robot.plot(traj.q, backend = 'pyplot', limits=[-0.25, 1.25, -0.5, 0.5, 0, 1], movie='panda_pyplot.gif')
    
def zadanie_2():
    pass # zastąp tę linię swoim kodem

def zadanie_3():
    pass # zastąp tę linię swoim kodem

# ...

# wykonywanie wybranej funkcji
if __name__ == '__main__':
    #przyklad_1()
    #...
    zadanie_1()
    #zadanie_2()
    #zadanie_3()