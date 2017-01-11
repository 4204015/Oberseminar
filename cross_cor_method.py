"""
-------------------------------------------------------------------------------
CROSS-CORRELATION METHOD: cross_cor_method.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
--------
"""
from matplotlib.pyplot import plot, grid, show, figure, stem

import numpy as np

def cross_cor_method(y,u,A,N,dt):
    
    R_uy = np.correlate(u,y,'full')# / len(u)
    # c_{av}[k] = sum_n u[n+k] * conj(y[n]) , Mode 'full'
    # c'_{av}[k] = sum_n u[n] * conj(y[n+k]) => c'_{av}[k] = c_{av}[-k]
    
    #Phi_inv = Phi[::-1]
    #Phi_mod = R_uy[len(u) - 1 : len(u) + N - 1]
    
    # Impulsantwort des Systems
    #g = 1 / (A**2 * dt) * (N/ (N + 1)) * (Phi_mod + sum(Phi_mod))
    
    g = R_uy
    G = np.fft.fft(g)
    
    
    return G, R_uy, g