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
    pass # zastąp tę linię swoim kodem

def zadanie_2():
    pass # zastąp tę linię swoim kodem

# ...

# wykonywanie wybranej funkcji
if __name__ == '__main__':
    przyklad_1()