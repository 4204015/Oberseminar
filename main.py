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
from matplotlib.pyplot import plot, grid, show, figure

# Library for symbolic mathematics
import sympy as sp

# Library for numeric mathematics
import numpy as np

# Own functions
from sim import Simulator
from area_method import area_method
from prbs_gen import prbsfnc
from cross_cor_method import cross_cor_method

# SYSTEM ----------------------------------------------------------------------

s = sp.Symbol('s')

#PT1
K = 1
T = 3
PT1 = K / (T*s + 1)

#PT2
K = 1
T = 3
theta = 0.3
PT2 = K / (T**2 * s**2 + 2*theta*T*s + 1)

# SIMULATIONSSTEUERUNG --------------------------------------------------------

t_max = 50                  # Simulationsdauer in Sekunden

t_sprung = 0
#u = stepfnc(t_sprung, 1)  # Eingangsfunktion mit Zeitpunkt, Sprunghöhe

ufnc, u, A, N = prbsfnc(50,2**10)        # PRBS Signal mit Amplitude, Periodendauer

PID = [3,1,1,5]             # Parameter des PID Reglers - T_i, T_d, T_n, K
t, b_out, G = Simulator(t_max,ufnc,PT1,True,*PID)
y = b_out[G]

# SYSTEMIDENTIFIKATION --------------------------------------------------------

System = ''

# PARAMETERIDENTIFIKATION -----------------------------------------------------

if System == 'PT1':
    K_e,T_e = area_method(t,t_sprung,b_out[G])
    print("K_e = " + str(K_e))
    print("T_e = " + str(T_e))
    
#elif System == 'PT2':
    
    
else:   # System -> unknown
    F = cross_cor_method(b_out[G],u,A,N)
    figure(0)
    plot(F[1:100].real,F[1:100].imag)
     

# AUSGABE ---------------------------------------------------------------------
figure(1)
plot(t, b_out[G])
grid()
show()
