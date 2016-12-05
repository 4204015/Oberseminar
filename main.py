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
from matplotlib.pyplot import plot, grid, show, figure, title, xlabel, ylabel, subplot, tight_layout, close

# Library for symbolic mathematics
import sympy as sp

# Library for numeric mathematics
import numpy as np

import scipy as sc

import control

# Own functions
from sim import Simulator
from area_method import area_method
from prbs_gen import prbsfnc
from cross_cor_method import cross_cor_method

close("all")

# SYSTEM ----------------------------------------------------------------------

s = sp.Symbol('s')

#PT1
K = 2
T = 3
PT1 = K / (T*s + 1)


#PT2
K = 1
T = 3
theta = 0.3
PT2 = K / (T**2 * s**2 + 2*theta*T*s + 1)
#G_ = control.tf([K],[T**2,2*theta*T,1])


# Process II [Hang1991]
#P2 = (1 - 0.5*s) / (1 + s)**3 
#dt = 0.4
#G_ = control.tf([-0.5,1],[1,3,3,1])


# Critical damped System [Husin2008]
CDS = 9 / (s**2 + 6*s + 9)
G_= control.tf([9],[2,6,9])
dt = 0.1

###
figure(0)
w = np.logspace(-3,3,1000)
real, imag, freq = control.nyquist(G_,w)
grid(True)
xlabel('Re(s)')
ylabel('Im(s)')
show()
###

# SIMULATIONSSTEUERUNG --------------------------------------------------------

t_sprung = 0
#ufnc = stepfnc(t_sprung, 1)  # Eingangsfunktion mit Zeitpunkt, Sprunghöhe

N = 63
ufnc, u_, A, N = prbsfnc(1,N,dt)        # PRBS Signal mit Amplitude, Periodendauer

PID = [3,1,1,5]             # Parameter des PID Reglers - T_i, T_d, T_n, K

t_max = int(2.5*dt*N)                  # Simulationsdauer in Sekunden
t, b_out, G, IN = Simulator(t_max,ufnc,CDS,True,*PID)
y = b_out[G]
u = np.array(b_out[IN])
#u -= 1

# SYSTEMIDENTIFIKATION --------------------------------------------------------

System = ''

# PARAMETERIDENTIFIKATION -----------------------------------------------------

if System == 'PT1':
    K_e,T_e = area_method(t,t_sprung,b_out[G])
    print("K_e = " + str(K_e))
    print("T_e = " + str(T_e))
    
#elif System == 'PT2':
    
    
else:   # System -> unknown
    F, Phi, g = cross_cor_method(b_out[G],b_out[IN],A,N,dt)
    t_inv = t[::-1]
    t_cor = np.vstack([t_inv[:len(t)-1]*-1,t])
    
    AKF_u = np.correlate(u,u,'full') / len(u)
    
    figure(1)
    grid()
    subplot(411)
    plot(t,u)
    title('PRBS Signal u (T = %4.2f)' % (N*dt))
    
    subplot(412)
    plot(t,y)
    title('Systemausgang y')
    
    subplot(413)
    plot(t,AKF_u[len(t)-1:])
    title('Autokorrelation uu')
 
    subplot(414)
    plot(t,Phi[len(t)-1:])
    title('Kreuzkorrelation')
    
    tight_layout()
    
    figure(2)
    plot(F[:].real,F[:].imag)
     

# AUSGABE ---------------------------------------------------------------------
#figure(3)
#plot(t, b_out[G])
#
#show()
