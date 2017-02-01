# -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 20:39:04 2017

@author: Administrator
"""

import sympy as sp
import numpy as np
import scipy as sc
import matplotlib.pyplot as mp

gen_poly = (7,4)
m = 7
N = 2**m - 1
R = np.zeros(m,dtype=int)
R[0] = 1

arrayOut = list()
    
for i in range(0, N):

    arrayOut.append(R[0])
    xor = int(R[gen_poly[0]-1] ^ R[gen_poly[1]-1])
    R = np.concatenate(([xor],R[0:m-1]))

prbs = np.array(arrayOut)*2 - 1

#mp.figure(0)
#mp.step(np.arange(1,len(arrayOut)+1),arrayOut)

u = np.concatenate((prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs,prbs))

uu = list()

for i in range(0,len(u)):
    j = 0
    while j <= 2:
        uu.append(u[i])
        j = j + 1
    
u = np.array(uu)
u = np.concatenate(((np.array([])),u))

R_uu = np.correlate(u,u,'full') / len(u)

#mp.figure(1)
#mp.plot(R_uu)
#
#L = len(u)
#S = np.fft.fft(u)    
#S = S[1:L/2+1]
#
#f = 1 * np.arange(0,len(S))/L
#
#mp.figure(2)
#mp.plot(f,abs(S))


L = len(R_uu)
Suu = np.fft.fft(R_uu)    
Suu = Suu[1:L/2+1]

f = 1 * np.arange(0,len(Suu))/L

mp.figure(3)
mp.plot(f,abs(Suu)/max(abs(Suu)))




