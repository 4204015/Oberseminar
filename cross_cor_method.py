"""
-------------------------------------------------------------------------------
CROSS-CORRELATION METHOD: cross_cor_method.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
--------
"""
import numpy as np

def cross_cor_method(y,u,A,N,dt):
    
    R_uu = np.correlate(u,u,'full') / len(u)
    
    R_uy_inv = np.correlate(u,y,'full') / len(u)
    # c_{av}[k] = sum_n u[n+k] * conj(y[n]) , Mode 'full'
    # c'_{av}[k] = sum_n u[n] * conj(y[n+k]) => c'_{av}[k] = c_{av}[-k]
    
    R_uy = R_uy_inv[::-1] # Kreuzkorrelation muss entsprechend der Implementierung und der Definition gespiegelt werden
    
    # Impulsantwort des Systems
    #g = 1 / (A**2 * dt) * (N/ (N + 1)) * (Phi_mod + sum(Phi_mod))
    
    idx = ((len(R_uy)-1) / 2)
    k = 2/(dt+0.1)
    g = R_uy[idx:]*k / R_uu[idx]

    G = np.fft.fft(g)
    
    
    return G, R_uy, g