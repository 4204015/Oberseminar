# -*- coding: utf-8 -*-

from pyblocksim import *

from prbs_gen import prbsfnc

from IPython import embed as IPS


f0 = 800
Lambda = 1/f0    # Taktzeit des PRBS Signals
#N = 10000
A = 1
ufnc, u_, N = prbsfnc(A,Lambda)        # PRBS Signal mit Amplitude, Periodendauer, Bitintervall



# parameters

m = 1.0
c = 10.0

u1, = inputs('u1') # external force and feedback

meas1 = Blockfnc(u1)
PT1 = TFBlock(1/(1+ s), u1) # 



u1fnc = ufnc

t, states = blocksimulation(10, (u1, u1fnc)) # simulate 10 seconds

bo = compute_block_ouptputs(states)

prbs1 = bo[meas1]
prbs2 = u_

K= 1999;
diff = prbs1[:K] - prbs2[:K]


# Nachweis, dass Eingangssignal das gewünschte war
assert np.count_nonzero(diff) == 0

if __name__ == "__main__":
    print("Vergleich erfolgreich")
    #IPS()

