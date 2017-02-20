from ode.rigid_body.rigid_body_dynamics import *
from ode.rigid_body.forces import *
from ode.rotation.rot import *
from ode.utils.plot_frame import *

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


    start_x         = np.zeros(3)
    rot_y           = Rot_y(-deg2rad(85.0))
    direction       = np.dot(rot_y,np.array([1,0,0],dtype=float))
    start_v         = direction * 10.0 # [m/s]


    # set initial conditions
    rigid_body.set_position(start_x)
    #rigid_body.set_velocity(start_v)

    # position + velocity + orientation
    record          = np.zeros((3 + 3 + 3,num_steps))

    record_real     = np.zeros((3,num_steps))

    Fg              = get_Fg(rigid_body.m,g=9.81)

    r = lambda t: start_x + (start_v * t) + (0.5 * np.array([0,0,-g]) * t**2)

    F = np.zeros(3,dtype=float)
    F0 = direction * 100 # [N]
    dt_F = 1/dt

    for i in range(0,num_steps):

        if i <= dt_F:
            F = Fg + F0
        else:
            F = Fg

        rigid_body.update(force=F,torque=np.zeros(3),dt=dt)
        record[0:3,i]    = rigid_body.get_position()
        record[3:6,i]    = rigid_body.get_velocity()
        record[6:9,i]    = rigid_body.get_orientation()
        record_real[:,i] = r(float(i) * dt)


    return record,start_v,record_real

if __name__ == '__main__':

  record,start_v,record_real = main()
  x     = record[0:3,:]
  v     = record[3:6,:]
  euler = record[6:9,:]

  line_v1 = np.stack(  (x[:,0],x[:,0] +  v[:,0])  )
  line_v2 = np.stack(  (x[:,0],x[:,0] + start_v)  )

  g               = 9.81
  start_x         = x[:,0]




  plt.close('all')
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
  ax.plot(line_v2[:,0], line_v2[:,1], line_v2[:,2], color='b')


  # plot simulation
  ax.scatter(x[0,0], x[1,0], x[2,0],color='g',s=100.0)
  ax.plot(x[0,:], x[1,:], x[2,:],color='r')
  ax.scatter(x[0,-1], x[1,-1], x[2,-1],color='r')

  # plot real trajectory
  #r = record_real
  #ax.plot(r[0,:], r[1,:], r[2,:], color='g')
  #ax.scatter(r[0,-1], r[1,-1], r[2,-1],color='g')

  RF_1.draw()
  RF_2.draw()

  ax.set_xlabel('x [m]')
  ax.set_ylabel('y [m]')
  ax.set_zlabel('z [m]')
  ax.set_xlim([-50,50])
  ax.set_ylim([-50,50])

  plt.show()


  #plt.show(block=False)

  #fig2 = plt.figure('velocity')
  #plt.hold(True)
  #plt.plot(v[0,:],'-k',label='x')
  #plt.plot(v[1,:],'--k',label='y')
  #plt.plot(v[2,:],':k',label='z')
  #print v
  #
