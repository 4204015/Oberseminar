"""
-------------------------------------------------------------------------------
PRBS GENERATOR: prbs_gen.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

import numpy as np
from random import choice

def prbsfnc(A,N,dt): 
    # choice wird mit der derzeitigen Systemzeit initialisiert
    prbs = np.array([choice([1,-1])*A for x in range(N)])             
    print(sum(prbs)/len(prbs))
    tt = [dt*x for x in range(N)]
    tt = np.array(tt)
    t_max = tt[N-1]
    
    def fnc(t):
        
        t = t % t_max
        idx = len(tt[tt < t])
        return prbs[idx] # + 1    # Überlagerung mit Einheitsprung    
        
    return fnc ,prbs , A, N