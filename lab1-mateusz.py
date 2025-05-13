# wczytanie potrzebnych podczas zajęć bibliotek:
import roboticstoolbox as rtb
import numpy as np
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
from matplotlib import pyplot as plt
# ...

# definicje funkcji:
def przyklad_1():
    # Złożenie macierzy rotacji
    R1 = SO3.Rx(0.3)
    R2 = SO3.Rz(30, 'deg')
    R3 = R1*R2
    # print(R1)
    # print(R2)
    # print(R3)

    # Wektor macierzy rotacji 
    angles = np.linspace(0, 2*np.pi, 30)
    R4 = SO3.Rx(angles)
    #print(R4)

    T1 = SE3(0.5, 1, 0.1) #Czysta translacja
    T2 = SE3.Ry(45, 'deg') #Czysta rotacja
    T3 = T1*T2
    #print(T3)

    #Inna metoda definiowania transformacji
    Rot = SO3.Ry(45, 'deg')
    Pos = np.array([0.5, 1, 0.1])
    T4 = SE3.Rt(Rot, Pos)
    #print(T4)

    #Wykresy
    R3.plot(frame='A', color='green', width=1)
    T3.plot(frame='B', color='blue', width=1)
    # plt.show()

    #Animacja
    T3.animate(frame='A', style='rviz', width=1, dims=[0,3], nframes=100)
    plt.show()

def zadanie_1():
    R = SO3.Rz(np.pi/-3)*SO3.Ry(np.pi/6)*SO3.Rx(np.pi/4)
    print(R)
    #Przy mnożeniu lewostronnym (od prawej do lewej) obracanie występuje wokół stałego układu odniesienia. Przy mnożeniu prawostronnym (od lewej do prawej) obracanie występuje wokół osi nowego układu odniesienia. 
    #Roll to obrót wokół osi x, Pitch to obrót wokół osi y, a yaw to obrót wokół osi z

    e1, e2, e3 = R.eul(unit='deg')
    #Uzyskane kąty dotyczną osi Z,Y,Z
    print('Kąty eulera: ',e1,' ',e2,' ',e3)
    e1, e2, e3 = R.eul()
    R1 = SO3.Eul([e1, e2, e3])
    print(R1)

    R.animate(frame='A', style='rviz', width=1, dims=[0,3], nframes=100)
    plt.show()

    det_R = np.linalg.det(R.A)
    inv_R = np.linalg.inv(R.A)
    T_R = np.transpose(R.A)

    n = R.A[:, 0]
    o = R.A[:, 1]
    a = R.A[:, 2]

    nTo = np.transpose(n)@o
    oTa = np.transpose(o)@a
    aTn = np.transpose(a)@n

    norm_n = np.linalg.norm(n)
    norm_o = np.linalg.norm(o)
    norm_a = np.linalg.norm(a)

    nxo = np.cross(n, o)
    oxa = np.cross(o, a)
    axn = np.cross(a, n)

    print('det(R) = ', det_R)
    print('R^(-1) = ', inv_R)
    print('R^T = ', T_R)

    print('n = ', n)
    print('o = ', o)
    print('a = ', a)
    
    print('n^T*o = ', nTo)
    print('o^T*a = ', oTa)
    print('a^T*n = ', aTn)

    print('norm(n) = ', norm_n)
    print('norm(o) = ', norm_o)
    print('norm(a) = ', norm_a)

    print('n x o = ', nxo)
    print('o x a = ', oxa)
    print('a x n = ', axn)

def zadanie_2():
    #Zmienne symboliczne
    l1, l2, l3, l4 = symbol('l1, l2, l3, l4')
    theta1, theta3 = symbol('theta1, theta3')
    d2 = symbol('d2')

    AB0 = SE3(0,0,l1)*SE3.Rx(np.pi/-2)
    A01 = SE3(0,0,l2)*SE3.Rz(theta1*-1)*SE3.Rx(np.pi/-2)
    A12 = SE3(0,0,d2)*SE3.Rz(np.pi/-2)*SE3(l3,0,0)
    A23 = SE3.Rz(theta3*-1)*SE3(l4,0,0)
    A33 = SE3.Rz(np.pi/2)*SE3.Rx(np.pi/2)

    print('AB0:')
    print(AB0)
    print('A01:')
    print(A01)
    print('A12:')
    print(A12)
    print('A23:')
    print(A23)
    print("A33':")
    print(A33)

def zadanie_3():
    dp = np.array([1, -2, 2])

    ABS = SE3.Rt(SO3.Rz(np.pi/2), np.array([2,5,0]))
    APS = SE3.Rt(SO3.Ry(np.pi), np.array([3,-6,4]))
    ASP = APS.inv()
    
    ds = ASP*dp
    db = ABS*ds
    print(db.flatten())


# wykonywanie wybranej funkcji
if __name__ == '__main__':
    zadanie_3()