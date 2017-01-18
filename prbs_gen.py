"""
-------------------------------------------------------------------------------
PRBS GENERATOR: prbs_gen.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

import numpy as np
import random as rd


def prbsfnc(A,Lambda): 
    
    gen_poly = (6,5);
    m = 6
    N = 2**m - 1
    R = np.zeros(m,dtype=int)
    R[0] = 1
    
    arrayOut = list()
        
    for i in range(0, N):
    
        arrayOut.append(R[0])
        xor = int(R[gen_poly[0]-1] ^ R[gen_poly[1]-1])
        R = np.concatenate(([xor],R[0:m-1]))
    
    prbs = np.array(arrayOut)*2 - 1
    
    print(sum(prbs)/len(prbs))
    tt = [Lambda*x for x in range(N)]
    tt = np.array(tt)
    t_max = tt[N-1]

    def fnc(t):
        
        t = t % t_max
        idx = len(tt[tt < t])
        return prbs[idx] # + 1    # Überlagerung mit Einheitsprung    
        
    return fnc ,prbs, N