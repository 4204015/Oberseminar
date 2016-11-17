import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.integrate import ode, simps
import sys

#------------------------------------------------------------------------------
# Globale Variablen
#------------------------------------------------------------------------------

# Simulationssteuerung:
dt = 0.01       # Schrittweite der Simulation
delta = 10**-6  # untere Schranke
null = 10**-16  # numerische Null

t = []          # Zeitvektor
y = []          # Simulationsergebnis
name = ''       # Name des simulierten Systems

t_sprung = 5.0      # Zeitpunkt des Sprungs in Sekunden
sprung_hoehe = 1.0  # Sprunghöhe
#------------------------------------------------------------------------------

def PT2(oscillatably):
    """
    Zustandsmatrizen eines PT2-Gliedes
    """
    K = 4    
    if oscillatably:
        # Schwingfall: 0 < theta < 1
        T = 5
        theta = 0.8
    else:
        # Kriechfall: theta > 1, Reihenschaltung zweier PT1-Glieder
        T1 = 5
        T2 = 3
        T = math.sqrt(T1 * T2)
        theta = (T1 + T2) / (2 * math.sqrt(T1 * T2)) 
                
    A = np.array([[0,1],[-1/(T**2),-2*theta/T]])
    B = np.array([0,K/(T**2)])
    C = np.array([[1,0]])
    
    func_name = sys._getframe().f_code.co_name
    
    return [A,B,C,func_name]

def PT1():
    """
    Zustandsmatrizen eines PT1-Gliedes G(s) = K / (Ts + 1)
    """
    T = 8
    K = 4
    
    A = np.array([[-1/T]])
    B = np.array([[K/T]])
    C = np.array([[1]])
    
    func_name = sys._getframe().f_code.co_name

    return [A,B,C,func_name]


def LTI(A,B,C,func_name):
      
    global name
    name = func_name
    
    def system(t,x):   
        u = 0 if t < t_sprung else sprung_hoehe
        x_dot = np.dot(A,x) + np.dot(B,u)
        return x_dot
            
    return [system, A.shape[1],C]
    

    
def Simulation(system, order,C):    
 
    global t
    global y
    # Neu initialisieren
    t = []   
    y = []
    
    x = np.zeros((1,order)) # Anfangswertvektor / Zustandsvektor

    #x[0,:] = 0
    r = ode(system).set_integrator('dopri5')
    r.set_initial_value(x[0,:],0)
    
    steadystate = False
    wendepunkt = False
    
    steigend = True
    first = True
    
    w = []  # Liste mit den Zeitpunkten der Wendepunkte mit f' = 0

    j = 0   # Hilfsvariable um Flachstelle von stationärem Endwert zu unterscheiden
    i = 0   # Schrittvariable für die Intergation
    while not(steadystate): 
        r.integrate(r.t + dt)
        x = np.concatenate((x,[r.y])) 
        
        # linksgekrümmt -> Steigung der Tangente nimmt zu
        # rechtsgekrümmt -> Steigung nimmt ab = Annäherung an Flachstelle
        k = 'linksgekrümmt' if np.diff(x,2) > 0 else 'rechtsgekrümmt'
        
        if k == 'linksgekrümmt':
            
            first = True
            
            if steigend and not(wendepunkt):
                wendepunkt = x[i+1,0] - x[i,0] < 0
    
            elif wendepunkt:
                steigend = False
                w.append(i-1)   # gefundener Wendepunkt wird vermerkt
                wendepunkt = False
                
            # Ist die Differenz zwischen zwei Wendepunkten kleiner als delta, ist der stationäre Endwert annähernd erreicht                           
            if len(w) > 1 and abs(x[w[len(w)-2],0] - x[w[len(w)-1],0]) < delta:
                steadystate = True 
                                         
        else: #rechtsgekrümmt
            # Prüfung ob eine Flachstelle reicht ist 
            if x[i,0] > null and abs(x[i+1,0] - x[i,0]) < delta and first:
                j = i *2 
                first = False
                        
            j = j - 1
            # Prüfung ob der stationäre Zustand bereits erreicht ist
            steadystate = True if j == 0 else False
                

        t.append(r.t)
        i += 1   
    
    t = np.array(t)
    
    # y = C * x + D * u  mit  D = 0 
    y = np.transpose(np.dot(C,np.transpose(x[1:,:])))

def area_method(t,y):
    """
    Methode der Modellbildung für die Regelstrecke, welche eine PT1 annimmt und
    über Flächenberechnungen die Zeitkonstante und den Verstärkungsfaktor schätzt.
    """
    # Schätzung (estimation) der Zeitkonstanten und K
    K_e = y[-1,0]
    T_e = (K_e*(t[-1] - t_sprung) - simps(y[:,0],t,dt)) / K_e
    
    print ("Geschätzte Parameter für das System \"%s\": " % (name))            
    print ("Zeitkonstante T_e = " + str(T_e))
    print ("Verstärkung K_e = " + str(K_e) + "\n")
    
    
def Ausgabe(t,y):
    plt.plot(t,y)
    plt.xlabel('Zeit in s')
    plt.ylabel('Ausgang')
    plt.title('Sprungantwort')

#------------------------------------------------------------------------------
# Simulation
#------------------------------------------------------------------------------
    
Simulation(*LTI(*PT1()))
Ausgabe(t,y)
area_method(t,y)

Simulation(*LTI(*PT2(True)))
Ausgabe(t,y)
area_method(t,y)
