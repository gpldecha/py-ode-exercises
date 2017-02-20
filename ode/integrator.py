# Euler's method to sloving a differential equation
import numpy as np

def euler_solve(func,y0,t0,tend,dt):
    """ Euler's solver

        Args:

            func        : differential equation dy/dt
            y0          : np.array initial value
            t0          : initial time
            tend        : final time
            dt          : time interval
    """
    Nsteps      = int(round(tend/dt))
    time        = int(t0)
    y           = y0
    num_states  = y0.shape[0]
    states = np.empty((2*num_states+1,Nsteps))

    for i in range(0,Nsteps):
        dy                      = func(time,y)
        y                       = y + dy * dt
        time                    = time + dt
        states[0:num_states,i]  = y
        states[num_states:-1,i] = dy
        states[-1,i]            = time

    return states
