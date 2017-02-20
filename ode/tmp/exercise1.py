#   Investigation of numerical approximation methods for first order ODE
#   [1] http://faculty.olin.edu/bstorey/Notes/DiffEq.pdf
#

#  dy/dt = -y
#      y = Ce^-t   (exact solution)

import numpy as np
import matplotlib.pyplot as plt


t = np.linspace(0,2,20)
y = np.exp(-t)

dydt = np.diff(y) / np.diff(t)


plt.figure()
plt.hold(True)
plt.plot(t,y)               # original function (solution of ODE)
plt.plot(t[1:],dydt,'--r')  # numerical derivative
plt.plot(t,-y)              # analytical derivative
plt.show()

def eulers_method():
    y       = 1     # initial condition
    dt      = 0.5
    time    = 0
    t_final = 2

    Nsteps  = int(round(t_final/dt))
    plt.figure()
    plt.hold(True)
    plt.plot(time,y,'*')

    print 'Nsteps: ', Nsteps

    # simulation loop
    for i in range(0,Nsteps):
        y    = y - dt *y
        time = time + dt
        plt.plot(time,y,'b*');

    t = np.linspace(0,t_final,100)
    y = np.exp(-t)

    plt.plot(t,y,'r')
    plt.show()


def main():

    eulers_method()


if __name__ == '__main__':
  main()
