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
    traj.plot(traj)

    traj = mtraj(trapezoidal, [0,2],[1,-1],50) #trajektoria trapezoidalna dla dwóch zmiennych konfiguracyjnych
    traj.plot(traj)

def zadanie_1():
    pass # zastąp tę linię swoim kodem
    
def zadanie_2():
    pass # zastąp tę linię swoim kodem

def zadanie_3():
    pass # zastąp tę linię swoim kodem

# ...

# wykonywanie wybranej funkcji
if __name__ == '__main__':
    przyklad_1()
    #...
    #zadanie_1()
    #zadanie_2()
    #zadanie_3()