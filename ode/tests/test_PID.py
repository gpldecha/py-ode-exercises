from ode.rigid_body.rigid_body_dynamics import *
from ode.rigid_body.forces import *
from ode.rotation.rot import *
from ode.utils.plot_frame import *
from ode.control.pid import *
from ode.control.models import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import linalg as LA



def main():

    print(' test rigid body dynamics')

    rigid_body      = RigidBodyDynamics()
    rigid_body.m    = 1.0
    dt              = 1.0/1000.0
    t               = 10.0 # time of simulation [s]
    g               = 9.81
    num_steps       = int(t / dt)

    rigid_body.set_position(np.zeros(3))

    record          = np.zeros((3 + 3 + 3,num_steps))
    record_pid      = np.zeros((6 + 6,num_steps))
    Fg              = get_Fg(rigid_body.m,g=9.81)

    config_pid      = {            # position | orientation
                        'Kp' : np.array([4,4,4,4,4,4],dtype=float),
                        'Ki' : np.array([0,0,0,0,0,0],dtype=float),
                        'Kd' : np.array([3,3,3,3,3,3],dtype=float),
                        'max':  50, # [N]
                        'min': -10, # [N]
                        'dt' : dt
                      }

    grav_model      = GravityModel(m = rigid_body.m,g=g)
    uC              = np.zeros(6)
    uC[0:3]         = grav_model.get_control()
    pid             = PID(config_pid)

    pid.add_model_constant(uC)
    pid.set_target(np.array([3,2,1,0,0,np.pi * 0.95],dtype=float))

    for i in range(0,num_steps):

        # controller
        r       = rigid_body.get_position()
        theta   = rigid_body.get_orientation()
        s       = np.append(r,theta)
        u       = pid.update(s)
        Fu      = u[0:3]    # force
        Tu      = u[3:6]    # torque

        F       = Fg + Fu
        T       = np.zeros(3) + Tu

        rigid_body.update(force=F,torque=T,dt=dt)


        record[0:3,i]       = rigid_body.get_position()
        record[3:6,i]       = rigid_body.get_velocity()
        record[6:9,i]       = rigid_body.get_orientation()
        record_pid[0:3,i]   = F
        record_pid[3:6,i]   = T
        record_pid[6:12,i]  = pid.e

    return record_pid, record

def plot_pid(record_pid,record_pos):

    Fu    = record_pid[0:3,:]
    Tu    = record_pid[3:6,:]
    Fe    = record_pid[6:9,:]
    Te    = record_pid[9:12,:]
    r     = record_pos[0:3,:]
    theta = record_pos[6:9,:]

    fig = plt.figure()

    """ Plot for Force """

    ax  = fig.gca
    plt.subplot(321)
    plt.plot(Fu[0,:],'-k',label='Fx')
    plt.plot(Fu[1,:],'--k',label='Fy')
    plt.plot(Fu[2,:],':k',label='Fz')
    plt.legend(loc="upper left", ncol=1, shadow=True, fancybox=True)
    plt.xlabel('t')
    plt.ylabel('[N]')
    plt.title('Control: Force',fontsize=18)

    plt.subplot(323)
    plt.plot(Fe[0,:],'-k',color='r',label='ex')
    plt.plot(Fe[1,:],'--k',color='r',label='ey')
    plt.plot(Fe[2,:],':k',color='r',label='ez')
    plt.legend(loc="upper left", ncol=1, shadow=True, fancybox=True)
    plt.title('Target error',fontsize=18)
    plt.xlabel('t')
    plt.ylabel('[m]')

    plt.subplot(325)
    plt.plot(r[0,:],'-k',color='b',label='x')
    plt.plot(r[1,:],'--k',color='b',label='y')
    plt.plot(r[2,:],':k',color='b',label='z')
    plt.legend(loc="upper left", ncol=1, shadow=True, fancybox=True)
    plt.title('Position',fontsize=18)
    plt.xlabel('t')
    plt.ylabel('[m]')


    """ Plot for Torque """

    plt.subplot(322)
    plt.plot(Tu[0,:],'-k',label='Ta')
    plt.plot(Tu[1,:],'--k',label='Tb')
    plt.plot(Tu[2,:],':k',label='Tg')
    plt.legend(loc="upper left", ncol=1, shadow=True, fancybox=True)
    plt.xlabel('t')
    plt.ylabel('[Nm]')
    plt.title('Control: torque',fontsize=18)

    ax = plt.subplot(324)
    plt.plot(Te[0,:],'-k',color='r',label='ea')
    plt.plot(Te[1,:],'--k',color='r',label='eb')
    plt.plot(Te[2,:],':k',color='r',label='eg')
    ax.set_yticks([-np.pi,-np.pi/2.0, 0, np.pi/2.0, np.pi])
    ax.set_yticklabels([r"$-\pi$", r"$-\frac{1}{2}\pi$",r"$0$", r"$\frac{1}{2}\pi$",r"$\pi$"],fontsize=20)
    plt.legend(loc="upper left", ncol=1, shadow=True, fancybox=True)
    plt.title('Target error',fontsize=18)
    plt.xlabel('t')
    plt.ylabel('[rad]')


    ax = plt.subplot(326)
    plt.plot(theta[0,:],'-k', color='b',label='x')
    plt.plot(theta[1,:],'--k',color='b',label='y')
    plt.plot(theta[2,:],':k', color='b',label='z')
    ax.set_yticks([-np.pi,-np.pi/2.0, 0, np.pi/2.0, np.pi])
    ax.set_yticklabels([r"$-\pi$", r"$-\frac{1}{2}\pi$",r"$0$", r"$\frac{1}{2}\pi$",r"$\pi$"],fontsize=20)
    plt.legend(loc="upper left", ncol=1, shadow=True, fancybox=True)
    plt.title('Orientation',fontsize=18)
    plt.xlabel('t')
    plt.ylabel('[rad]')


    return fig


def plot_trajectory(record,start_v):
    x     = record[0:3,:]
    v     = record[3:6,:]
    euler = record[6:9,:]

    line_v1 = np.stack(  (x[:,0],x[:,0] +  v[:,0])  )

    g               = 9.81
    start_x         = x[:,0]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    dim  = 5
    X, Y = np.meshgrid([-dim, dim], [-dim, dim])
    Z = np.zeros((2, 2))

    RF_1 = Plot_frame(ax)
    RF_2 = Plot_frame(ax)

    RF_1.update_pos(x[:,0])
    RF_1.update_rot(euler[:,0])

    RF_2.update_pos(x[:,-1])
    RF_2.update_rot(euler[:,-1])


    fig.hold(True)
    ax.plot_surface(X, Y, Z, color='green', alpha=.5, linewidth=0, zorder=1)
    ax.plot(line_v1[:,0], line_v1[:,1], line_v1[:,2], color='r')

    # plot simulation
    ax.scatter(x[0,0], x[1,0], x[2,0],color='g',s=100.0)
    ax.plot(x[0,:], x[1,:], x[2,:],color='r')
    ax.scatter(x[0,-1], x[1,-1], x[2,-1],color='r')

    RF_1.draw()
    RF_2.draw()

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_xlim([-50,50])
    ax.set_ylim([-50,50])

    return fig


if __name__ == '__main__':

  record_pid, record = main()

  plt.close('all')

  plot_pid(record_pid,record)

  plt.show()
