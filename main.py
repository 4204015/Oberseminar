"""
-------------------------------------------------------------------------------
MAIN SIMULATION PROGRAM: main.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

# IMPORT ----------------------------------------------------------------------

# Module for block based modeling and simulation of dynamic systems
from pyblocksim import inputs, TFBlock, stepfnc, blocksimulation, compute_block_ouptputs

# Plotting library
from matplotlib.pyplot import plot, grid, show

# Library for symbolic mathematics
import sympy as sp

# Library for numeric mathematics
import numpy as np

# Own functions
from sim import Simulator
from area_method import area_method
from prbs_gen import prbsfnc

# SYSTEM ----------------------------------------------------------------------

s = sp.Symbol('s')

#PT1
K = 1
T = 3
PT1 = K / (T*s + 1)


# SIMULATIONSSTEUERUNG --------------------------------------------------------

t_max = 10                  # Simulationsdauer in Sekunden

t_sprung = 0
#u = stepfnc(t_sprung, 45)    # Eingangsfunktion mit Zeitpunkt, Sprunghöhe

u = prbsfnc(10,5)

PID = [3,1,1,5]             # Parameter des PID Reglers - T_i, T_d, T_n, K
t, b_out, G = Simulator(t_max,u,PT1,True,*PID)
y = b_out[G]

# SYSTEMIDENTIFIKATION --------------------------------------------------------

System = 'PT1'

# PARAMETERIDENTIFIKATION -----------------------------------------------------

if System == 'PT1':

    K_e,T_e = area_method(t,t_sprung,b_out[G])
    
#elif System == 'PT2':
    
    
#else:   # System -> unknown
    

# AUSGABE ---------------------------------------------------------------------

print("K_e = " + str(K_e))
print("T_e = " + str(T_e))
plot(t, b_out[G])
grid()
show()
