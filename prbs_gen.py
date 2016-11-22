"""
-------------------------------------------------------------------------------
PRBS GENERATOR: prbs_gen.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

import numpy as np

def prbsfnc(A,N):
    global prbs
    prbs = A * (np.random.rand(N) - 0.5)     
    print(sum(prbs)/len(prbs))
    
    def fnc(t):
        global prbs            
        u = prbs[0]
        prbs = np.roll(prbs,1)  # Liste mit PRB-Zahlen wird rotiert
        #print (u)
        return u   
        
    return fnc ,prbs , A, N