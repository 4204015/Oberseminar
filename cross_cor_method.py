"""
-------------------------------------------------------------------------------
CROSS-CORRELATION METHOD: cross_cor_method.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
--------
"""

import numpy as np

def cross_cor_method(y,u,A,N):
    
    dt = 5e-3
    Phi = np.correlate(u,y[1:],'valid')
    # Mode 'valid' returns output of length max(M,N) -  min(M,N) - 1

    
    # Impulsantwort des Systems
    g = 1 / (A**2 * dt) * (N/ (N + 1)) * (Phi + sum(Phi)) 
    G = np.fft.fft(g)
    
    return G