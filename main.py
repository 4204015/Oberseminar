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
import matplotlib as mp

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
G_ = control.tf([K],[1,T])
figure(100)
control.bode(G_)

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

dt = 0.5
N = 5000 #int((1.5 * 15) / dt)
a = 6
ufnc, u_, A, N = prbsfnc(a,N,dt)        # PRBS Signal mit Amplitude, Periodendauer, Bitintervall

PID = [3,1,1,5]             # Parameter des PID Reglers - T_i, T_d, T_n, K

#t_max = int(1.5*dt*N)  
t_max = 60               # Simulationsdauer in Sekunden
t, b_out, G, IN, G_noise = Simulator(t_max,ufnc,PT1,True,*PID,True)
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
    
    K_e,T_e = area_method(t,t_sprung,b_out[G_noise])
    print("K_e = " + str(K_e))
    print("T_e = " + str(T_e))
    
#elif System == 'PT2':
    
    
else:   # System -> unknown
#    F, R_uy, g = cross_cor_method(b_out[G],b_out[IN],A,N,dt)
#    t_inv = t[::-1]
#    t_cor = np.vstack([t_inv[:len(t)-1]*-1,t])
    
    
#    figure(1)
#    grid()
#    subplot(411)
#    plot(t,u)
#    title('PRBS Signal u (T = %4.2f)' % (N*dt))
#    
#    subplot(412)
#    plot(t,y)
#    title('Systemausgang y')
#    
#    subplot(413)
#    plot(t,R_uu[len(t)-1:])
#    title('Autokorrelation R_uu')
#    xlabel('t in s')
# 
#    subplot(414)
#    #plot(t,Phi[len(t)-1:])
#    plot(R_uy)
#    title('Kreuzkorrelation R_uy')
#    
#    tight_layout()
    
    figure()
    #plot(t,g)
    np.linspace()
    g_a = K / T * np.exp(-t/T)
    plot(t,g_a)
    F = np.fft.fft(g_a)
    
    figure(2)
    plot(F[:].real,F[:].imag)
     
    L = (len(F) + 1 )/ 2  # Länge des halben FFT-Ergebnis
    fft_timestep = 5e-3  
    freq_axis = np.linspace(0, 1 / (2 * fft_timestep),L,endpoint=True)
    F = F * 2 / L   # Amplitude anpassen

    
    # Bodediagramm aus der ermittelten Übertragungsfunktion F
    F_abs = np.abs(F[:L])
    #F_phi = np.arctan2(F[:L].imag, F[:L].real) * 180 / np.pi
    F_phi = np.angle(F[:L], deg=True)  
                       
    figure(100)
    subplot(211)
    loglog(freq_axis,F_abs)
    title('Amplitudengang')
    
    subplot(212)
    semilogx(freq_axis,F_phi)
    title('Frequenzgang')

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
