"""
-------------------------------------------------------------------------------
REINISCH: reinisch.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

import numpy as np

def param(T_e, K_e):
    
    mu = len(T_e)
    
    a = 0.18
    c = 1.4
        
    T_sum = sum(T_e[1:4]) - T_e[0]
      
    T = list()            
                
    for k in range(3,mu-1):
        T.append(sum(T_e[k+1:mu]) * T_e[k])
                 
    b = sum(T) - T_e[0] * sum(np.array(T_e[3:mu]) - T_e[0])            
                
    a_k = a + c * (b / T_sum**2)
    
    T_i = sum(T_e[1:3])
    T_d = T_e[1]*T_e[2] / T_i
    T_n = T_d / 10 
    K = T_i / (a_k  * K_e * T_sum)
    
    return [T_i,T_d,T_n,K]