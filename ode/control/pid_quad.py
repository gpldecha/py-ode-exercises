"""
    PID controller
"""

import numpy as np


class PID(object):

    def __init__(self,config):
        """
            Args:

                Kx : np.array(D),   gain parameters of dimension D which is
                                    the size of the signal.
        """

        self.Kp     = config['Kp']
        self.Ki     = config['Ki']
        self.Kd     = config['Kd']
        self._dt    = config['dt']
        self.m      = config['m']
        self.g      = config['g']

        assert self.Kp.size == self.Ki.size == self.Kd.size

        self.e              = np.zeros(6)
        self.e_tmp          = np.zeros(6)
        self.P              = np.zeros(6)
        self.D              = np.zeros(6)
        self.I              = np.zeros(6)
        self._derivative    = np.zeros(6)
        self._integral      = np.zeros(6)

    def update(self,x,v):
        """
            Args:
                x   : np.array(3), current position.
                v   : np.array(3), current velocity.

            Returns:
                u : np.array(D), control output
        """
        self.e              = self.target - y
        self._derivative    = (self.e - self.e_tmp) / self._dt
        self._integral      = self._integral + self.e * self._dt

        self.P              = self.Kp * self.e
        self.D              = self.Kd * self._derivative
        self.I              = self.Ki * self._integral

        return self.P + self.I + self.D


    def set_target(self,x):
        """ target x

            Args:
                x : np.array(3), Cartesian position

        """
        self.target     = np.array([x[0],x[1],x[2],0,0,0],dtype=float)
        self.integrator = 0.0
        self.derivator  = 0.0
