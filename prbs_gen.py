"""
-------------------------------------------------------------------------------
PRBS GENERATOR: prbs_gen.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

import random

def prbsfnc(A,N):
    prbs = []
    for num in range(N):  
        prbs.append(random.uniform(0, A))
        
    def fnc(t):
        size = N
        dt = 5e-3
        i = int(t // dt) - 1
        
        while i > size:
            i = i - size
        
        if i > -1 and i < size:
            u = prbs[i]

        else:
            u = 0
            
        return u      
    return fnc