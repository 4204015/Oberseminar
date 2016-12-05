import numpy as nu

def ident_MKQ(u, y, dt, K_0, T_0)
    
    #for i=0 in range(u):
        #dy[i]=(K_0*u[i]-y[i])/T_0;
    dy=y.diff(dt) #difference(y)
    
    phi = nu.matrix ([-dy, u])
    
    inv_phi=nu.transpose(phi)*nu.invers(phi*nu.transpose(phi))
    
    p = inv_phi*y
    
    return p