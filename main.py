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
from matplotlib.pyplot import plot, grid, show, figure, title, xlabel, ylabel, subplot, tight_layout, ylim, xlim, close, loglog, semilogx
import matplotlib.pyplot as mp

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

# PT1
K = 1
T = 1
PT1 = K / (T*s + 1)
o = 1

#
##PT2
#K = 1
#T = 3
#theta = 0.3
#PT2 = K / (T**2 * s**2 + 2*theta*T*s + 1)
##G_ = control.tf([K],[T**2,2*theta*T,1])
#
#
## Process II [Hang1991]
#P2 = (1 - 0.05*s) / (1 + s)**3 
#dt = 0.02
#G_ = control.tf([-0.5,1],[1,3,3,1])
#figure(100)
#control.bode(G_)

# Critical damped System [Husin2008]
#CDS = 9 / (s**2 + 6*s + 9)
#G_= control.tf([9],[2,6,9])
#dt = 0.1

###
#figure(0)
#w = np.logspace(-3,3,1000)
#real, imag, freq = control.nyquist(G_,w)
#grid(True)
#xlabel('Re(s)')
#ylabel('Im(s)')
#xlim(-0.5,0.1)
#ylim(-0.7,0.2)
#show()
###


# 
#P = (1 - 0.05*s + 4* s**2) / (1 + s)**3 
#G_ = control.tf([4,-0.5,1],[1,3,3,1])
#figure(100)
#control.bode(G_)


# SIMULATIONSSTEUERUNG --------------------------------------------------------

t_sprung = 5
#ufnc = stepfnc(t_sprung, 1)  # Eingangsfunktion mit Zeitpunkt, Sprunghöhe

f0 = 800
Lambda = 1/f0    # Taktzeit des PRBS Signals
#N = 10000
A = 1
ufnc, u_, N = prbsfnc(A,Lambda)        # PRBS Signal mit Amplitude, Periodendauer, Bitintervall

PID = [3,1,1,5]         # Parameter des PID Reglers - T_i, T_d, T_n, K
 

dt = 1e-4               # Schrittweite des Ergebnisvektors
Fa = 1/dt                # Abtastfrequenz
#t_max = 100             # Simulationsdauer in Sekunden
t_max = Lambda * N * 4 
t, b_out, S, IN, S_noise = Simulator(dt,t_max,ufnc,PT1**o,True,*PID,True)
y = b_out[S]
u = np.array(b_out[IN])
#u -= 1

# SYSTEMIDENTIFIKATION --------------------------------------------------------

System = ''

# PARAMETERIDENTIFIKATION -----------------------------------------------------

if System == 'PT1':
    K_e,T_e = area_method(t,t_sprung,b_out[S])
    print("K_e = " + str(K_e))
    print("T_e = " + str(T_e))
    
    K_e,T_e = area_method(t,t_sprung,b_out[S_noise])
    print("K_e = " + str(K_e))
    print("T_e = " + str(T_e))
    
#elif System == 'PT2':
    
    
else:   # System -> unknown
    G, w, g = cross_cor_method(y,u,A,N,Lambda,t,o,dt)

    # "Analytisches" Bodediagramm
    PT1 = control.tf([K],[T,1])
    [mag,phase,wout] = control.bode(PT1**o,w,Plot=False)

    
    # Bodediagramm aus der ermittelten Übertragungsfunktion G
    G_abs = abs(G)
    G_phi = np.angle(G,deg = True)  
    
    ####
    PT1 = control.tf([K],[T,1])
    g_a, tout = control.matlab.impulse(PT1**o,t[0:len(g)])
    L = len(g_a)
    G_ = np.fft.fft(g_a)   
    G_ = dt*G_[1:L/2+1]
    G_abs_a = abs(G_)
    G_phi_a = np.angle(G_,deg = True)
    ####
    
    figure(2)
    subplot(211)
    semilogx(w,20*np.log10(G_abs_a),'r')
    semilogx(wout,20*np.log10(mag),'g')
    semilogx(w,20*np.log10(G_abs),'--b')
    mp.axvline(x = 2*np.pi*f0, color='y')
    title('Amplitudengang')
    ylabel('Amplitude in dB')
    
    subplot(212)
    semilogx(w,G_phi_a,'r')
    semilogx(wout,phase,'g')
    semilogx(w,G_phi,'--b')
    mp.axvline(x = 2*np.pi*f0, color='y')
    title('Frequenzgang')
    ylabel('Phase in Grad')
    
    show()
    
    figure()
    plot(G_.real,G_.imag,'r')
    plot(G.real,G.imag,'b')
    
    
    
    L = len(u)
    S = np.fft.fft(u)    
    S = S[1:L/2+1]
    f = 1/dt * np.arange(0,len(S))/L
    figure(5)
    plot(f,abs(S))

    
    
    i = 0
    while 1:
        i = i + 1
        if G[i].imag**2 < 0.001:
            T_u_mess = 2*np.pi / (w[i])
            K_u_mess = 1/abs(G[i].real)
            break
    j = 0
    while 1:
        j = j + 1
        if G_[j].imag**2 < 0.001:
            T_u_ana = 2*np.pi / (w[j])
            K_u_ana = 1/abs(G_[i].real)
            break        

# AUSGABE ---------------------------------------------------------------------

#mp.rcParams.update({'font.size': 30})
#figure(3)
#subplot(121)
#plot(t, b_out[G])
#xlabel('t in s')
#ylabel('y')
#ylim(0,2.2)
#subplot(122)
#plot(t, b_out[G_noise])
#xlabel('t in s')
#ylabel('y')
#ylim(0,2.2)
#
#show()
