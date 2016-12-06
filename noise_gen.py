"""
-------------------------------------------------------------------------------
NOISE GENERATOR: noise_gen.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""
import numpy as np

def noise_fnc():   
    A = 0.5
    N = 10000
    global noise
    noise = A * (np.random.rand(N) - 0.5)
    
    def fnc(t):
        global noise
        n = noise[0]
        noise = np.roll(noise,1)
        return n  
        
    return fnc