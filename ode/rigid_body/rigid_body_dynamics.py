""" Equation of motion of a point mass particle


    Y(t) = [ x(t), v(t) ]^T

    - to simulate a particle we have to know the force F(t) acting
      on it.

      F(t) = gravity, wind, etc...

      dY(t)/dt = [ v(t), F(t)/m ]^T

"""
import numpy as np
from numpy.linalg import inv
from ode.rotation.rot import *
import warnings

def get_default_rigid_body_param():
    config = {
            'mass'      : 2.0,              # Kg
            'inertia'   : np.identity(3),   # I
            'x'         : np.zeros(3),      # position
            'R'         : np.identity(3),   # orientation
            'P'         : np.zeros(3),      # linear momentum
            'L'         : np.zeros(3),      # angular momentum
        }
    return config



class RigidBodyDynamics(object):

    def __init__(self,config=get_default_rigid_body_param()):
        """ Constant quantities """
        self.m          = config['mass']
        self.I_b        = config['inertia'] # in body frame of reference
        self.invI_b     = inv(self.I_b)

        """ State variables """
        self.x          = config['x']
        self.R          = config['R']
        self.P          = config['P']
        self.L          = config['L']

        """ Derived quantities """
        self.v          = self.P / self.m
        self.Inv        = np.dot(self.R,np.dot(self.invI_b,self.R.T))
        self.omega      = np.dot(self.Inv,self.L)

        """ Computed quantities """
        self.force      = np.zeros(3)   # Controller provides these
        self.torque     = np.zeros(3)   # quantaties.

        """ tmp """
        self.W          = np.zeros((3,3))
        self.dotR       = np.zeros((3,3))

    def omega2star(self):
        self.W[0][1] = -self.omega[2]
        self.W[0][2] =  self.omega[1]

        self.W[1][0] =   self.omega[2]
        self.W[1][2] =  -self.omega[0]

        self.W[2][0] =  -self.omega[1]
        self.W[2][1] =   self.omega[0]

    def set_position(self,position):
        self.x = position

    def set_velocity(self,velocity):
        self.v  = velocity
        self.P  = self.m * self.v

    def set_orientation(self,*args):
        if len(args) == 1:
            eulers = args[0]
            self.R = Rot_zyx(eulers[0],eulers[1],eulers[2])
        elif len(args) == 3:
            self.R = Rot_zyx(args[0],args[1],args[2])
        else:
            warnings.warn('Wrong number of arguments, either set_orientation([alpha,beta,gamma]) or set_orientation(alpha,beta,gamma)')
        self.Inv        = np.dot(self.R,np.dot(self.invI_b,self.R.T))
        self.omega      = np.dot(self.Inv,self.L)

    def get_orientation(self):
        return Rot2Euler(self.R)

    def get_position(self):
        return self.x

    def get_velocity(self):
        return self.v


    def dydt(self):
        """ dY(t)/dt of Rigid body

            Args:

                Y(t)     : [ x, R, P, L ], state vector

            Returns:
                dY(t)/dt :  [v, dR, dP, dL], derivative of state vector
        """

        self.omega2star() # populates W
        self.dotR = np.dot(self.W,self.R)

        return self.v,self.dotR,self.force,self.torque

    def update(self,force,torque,dt):
         """ update the state of the rigid body given
             a force and torque.

             Args:

                force  : np.array(3)
                torque : np.array(3)
         """
         self.force  = force
         self.torque = torque

         dx, dR, dP, dL = self.dydt()

         self.x = self.x + dx * dt
         self.R = self.R + dR * dt
         self.P = self.P + dP * dt
         self.L = self.L + dL * dt

         self.v      = self.P / self.m
         self.Inv    = np.dot(self.R,np.dot(self.invI_b,self.R.T))
         self.omega  = np.dot(self.Inv,self.L)
