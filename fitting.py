"""
-------------------------------------------------------------------------------
FITTING: fitting.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

import numpy as np
from scipy.optimize import curve_fit
from matplotlib.pyplot import plot, grid, show, figure, title, xlabel, ylabel, subplot, tight_layout, ylim, xlim, close, loglog, semilogx

def fit(G_abs,G_phi,w,f0,sys):
    
    idx = len(w[w < (f0*0.5*np.pi)])
    
    w = w[0:idx]
    G_abs = G_abs[0:idx]
    G_phi = G_phi[0:idx]

#    figure(20)
#    subplot(211)
#
#    semilogx(w,20*np.log10(G_abs))
#    title('Amplitudengang')
#    ylabel('Amplitude in dB')
#    
#    subplot(212)
#    semilogx(w,G_phi)
#    title('Frequenzgang')
#    ylabel('Phase in Grad')
#    show()
    
    if sys == 'PT1':

        def betrag(w,T,K):

            return K / np.sqrt(1+(T*w)**2)
        
        def phase(w,T):
    
            return - np.arctan2(T*w,1)
          
    p1, pcov = curve_fit(betrag,w,G_abs)
    p2, pcov = curve_fit(phase,w,np.radians(G_phi))
    
    return p1,p2