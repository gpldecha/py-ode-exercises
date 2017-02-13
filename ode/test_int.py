from integrator import *
import matplotlib.pyplot as plt


def main():
    dydy = lambda t,y: -y
    states = euler_solve(dydy,1,t0=0,tend=1,dt=0.01)

    t  = states[0,:]
    y  = states[1,:]
    dy = states[2,:]

    plt.figure()
    plt.hold(True)
    plt.plot(t,y)
    plt.show()



if __name__ == '__main__':
  main()
