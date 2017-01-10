"""
-------------------------------------------------------------------------------
PRBS GENERATOR: prbs_gen.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

import numpy as np
import random as rd

variante = 'one'

def prbsfnc(A,N,dt): 
    
    """
    Variante 1
    """
    
    # choice wird mit der derzeitigen Systemzeit initialisiert
    if variante == 'one':
        rd.seed(123)
        prbs = np.array([rd.choice([1,-1])*A for x in range(N)])             
    
    """
    Variante 2
    --> http://blacbird.de/wp-content/uploads/2016/09/iPython_PRBS-Generator.html
    """
    
    if variante == 'two':
        generatorPolynom = 0x0001
        xOr_1 = 4
        xOr_2 = 0
        length = 9
    
        arrayOut = list()
    
        for i in range(0, 510):
            #Compute first shift value
            out = ((generatorPolynom >> xOr_1) & 1) ^ ((generatorPolynom >> xOr_2) & 1)
        
            #output logic
            arrayOut.append(generatorPolynom & 1)
            
            #shift generator Polynom one the right and append out
            generatorPolynom = ((generatorPolynom >> 1) | (out << (length-1))) 
    
        prbs = np.array(arrayOut)*2 - 1

    """
    """

    print(sum(prbs)/len(prbs))
    tt = [dt*x for x in range(N)]
    tt = np.array(tt)
    t_max = tt[N-1]

    def fnc(t):
        
        t = t % t_max
        idx = len(tt[tt < t])
        return prbs[idx] # + 1    # Überlagerung mit Einheitsprung    
        
    return fnc ,prbs , A, N