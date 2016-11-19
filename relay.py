# -*- coding: utf-8 -*-
"""
Created on Sat Nov 19 10:57:52 2016

@author: Philipp
"""

def relay (u):
    # Definition der Schranken
    ha = 0.065;
    he = 0.085;
    # Defaultwert fuer y festsetzen
    y = 0;

    if (u >= he):
        y = 1

    if (u >= ha) and (u < he) and (x == 1):
        y = 1

    if (-ha <= u < ha):
        y = 0

    if (u <= -he):
        y = -1

    if (-he < u) and (u <= -ha) and (x == -1):
        y = -1   
    
    return y