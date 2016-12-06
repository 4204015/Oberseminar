"""
-------------------------------------------------------------------------------
AREA METHOD: area_method.py
-------------------------------------------------------------------------------
Oberseminar Regelungstechnik - Auto-tuning PID
-------------------------------------------------------------------------------
"""

from scipy.integrate import simps
import numpy as np

def area_method(t,t_sprung,y):
    """
    Methode der Modellbildung für die Regelstrecke, welche eine PT1 annimmt und
    über Flächenberechnungen die Zeitkonstante und den Verstärkungsfaktor schätzt.
    """
    # dt = t[t.shape[0]-1,0] / t.shape[0]    # Achtung, feste Schrittweite wird angenommen
    y = np.array(y)
    t = t[1:,0]
    y = y[1:]    

    # Schätzung (estimation) der Zeitkonstanten und K
    K = sum(y[-100:-1])/99
    T = (K*(t[-1] - t_sprung) - simps(y,t)) / K
    
    return K,T