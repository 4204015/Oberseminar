"""
-------------------------------------------------------------------------------
CROSS-CORRELATION METHOD: cross_cor_method.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
--------
"""
from matplotlib.pyplot import plot, grid, show, figure, title, xlabel, ylabel, subplot, tight_layout, ylim, xlim, close, loglog, semilogx
import numpy as np
import control

def cross_cor_method(y,u,A,N,dt,t,o):
    
    PT1 = control.tf([1],[1,1])
    g_a, tout = control.matlab.impulse(PT1**o,t)
    
    """ Bestimmung der Kreukorrelation -> Gewichtsfunktion """
    
    R_uu = np.correlate(u,u,'full') / len(u)
    
    R_uy_inv = np.correlate(u,y,'full') / len(u)
    # c_{av}[k] = sum_n u[n+k] * conj(y[n]) , Mode 'full'
    # c'_{av}[k] = sum_n u[n] * conj(y[n+k]) => c'_{av}[k] = c_{av}[-k]
    
    R_uy = R_uy_inv[::-1] # Kreuzkorrelation muss entsprechend der Implementierung und der Definition gespiegelt werden
    
    
    idx = ((len(R_uy)) / 2)
#   k = 2/(dt+0.1)   
#   g = R_uy[idx:]*k / R_uu[idx]
    
    g = 1/(A**2 * ((N+1)/N) * dt) * (R_uy[idx:] + A**2 / N)

    """ Bestimmung der Frequenzantwort aus der Gewichtsfunktion """
    
    # FFT-Parameter
    dt = 5e-3   # Zeitauflösung
    Fs = 1/dt   # Abtastfrequenz
    L = len(g)
    
    G = np.fft.fft(g)    
    G = dt*G[1:L/2+1]
    
    # Frequenzachse
    w = 2 * np.pi * Fs * np.arange(0,(L/2)-1)/L
     
    
    """ Ausgabe Übersicht """
    
    figure(1)
    grid()
    subplot(511)
    plot(t,u)
    title('PRBS Signal u')
    
    subplot(512)
    plot(t,y)
    title('Systemausgang y')
    
    subplot(513)
    plot(t,R_uu[len(t)-1:])
    title('Autokorrelation R_uu')
    xlabel('t in s')
 
    subplot(514)
    plot(R_uy)
    title('Kreuzkorrelation R_uy')
    
    subplot(515)
    plot(t,g)
    plot(tout,g_a,'--r')
    title('Gewichtsfunktion')
    
    tight_layout()
    
    
    return G, w, g