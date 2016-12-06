"""
-------------------------------------------------------------------------------
SIMULATOR: sim.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

from pyblocksim import inputs, TFBlock, stepfnc, blocksimulation, compute_block_ouptputs, Blockfnc, loop
import sympy as sp
s = sp.Symbol('s')

# Own functions
from noise_gen import noise_fnc


def Simulator(t_max,u_func,controlled_system,only,T_i, T_d, T_n,K,noise):
    
    n_func = noise_fnc()
    
    u, y, n = inputs('u, y, n')
 
    if only == True:   # only controlled system
    
        G = TFBlock(controlled_system, u)
        
        IN = Blockfnc(u)
            
        SUM = Blockfnc(G.Y + n)
        
        G_noise = TFBlock((1 / (0.005*s + 1)), SUM.Y)    
        
        
#    else:    # simulation with PID
#        SUM1 = Blockfnc(u - y)
#        
#        I = TFBlock(1/(T_i * s), SUM1.Y)
#        
#        D = TFBlock(T_d*s / (T_n*s + 1), SUM1.Y)
#        
#        Kp = Blockfnc ((SUM1.Y + I.Y + D.Y)*K)
#        
#        G = TFBlock(controlled_system, Kp.Y)
#        
#        loop(G.Y, y)
#        
#        IN = Blockfnc(u)
    
    t, states = blocksimulation(t_max, {u: u_func, n: n_func})
    
    b_out = compute_block_ouptputs(states)
    
    return t,b_out, G, IN, G_noise


