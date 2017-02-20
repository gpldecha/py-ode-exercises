from integrator import *
import matplotlib.pyplot as plt
import numpy as np


def first_order_example():
    dydy = lambda t,y: -y
    states = euler_solve(dydy,1,t0=0,tend=1,dt=0.01)

    t  = states[0,:]
    y  = states[1,:]
    dy = states[2,:]

    plt.figure()
    plt.hold(True)
    plt.plot(t,y)
    plt.show()

def dynamics_spring_mass(x,v,k,m):
    """ 1st order reformulation of 2nd order spring-mass
        dynamical system. Returns the derivative dy/dt
        at a specific point y.

        Args:

            x     : numeric, 1D position of mass
            v     : numeric, 1D velocity of mass
            k     : numeric, spring coefficient
            m     : numeric, weight of mass

        Returns:

            dx/dt : numeric, velocity
            dv/dt : numeric, acceleration
    """
    return np.array([v,-k/m * x])



def second_oder_example(k,m,x,v):

    y = np.array([x,v])

    dydt = lambda t,y: dynamics_spring_mass(y[0],y[1],k,m)

    states = euler_solve(dydt,y,t0=0,tend=30,dt=0.001)

    return states




def main():

    k = 1.0
    m = 1.0

    # initial conditions
    x = 1
    v = 0

    states = second_oder_example(k,m,x,v)
    t = states[-1,:]
    x = states[0,:]

    # analytical solution to spring-mass system
    true_dynamics = lambda t: np.cos(t * k / m)

    xtrue = true_dynamics(t)

    plt.figure()
    plt.hold(True)
    plt.plot(t,x,'-b')
    plt.plot(t,xtrue,'-r')
    plt.show()




if __name__ == '__main__':
  main()
