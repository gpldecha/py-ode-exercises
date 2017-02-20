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
        # set minimum and maximum allowed control output
        if 'max' in config:
            self._max   = config['max']
            self._min   = config['min']
            self.bclip  = True
        else:
            self.bclip  = False

        assert self.Kp.size == self.Ki.size == self.Kd.size

        self.e              = np.zeros(self.Kp.size)
        self.e_tmp          = np.zeros(self.Kp.size)
        self.P              = np.zeros(self.Kp.size)
        self.D              = np.zeros(self.Kd.size)
        self.I              = np.zeros(self.Ki.size)
        self._derivative    = np.zeros(self.Ki.size)
        self._integral      = np.zeros(self.Ki.size)
        self._uC            = np.zeros(self.Ki.size)



    def add_model_constant(self,C):
        self._uC = self._uC + C

    def update(self,y):
        """
            Args:
                y : np.array(D), current state

            Returns:
                u : np.array(D), control output
        """
        self.e              = self.target - y
        self._derivative    = (self.e - self.e_tmp) / self._dt
        self.e_tmp          = self.e
        self._integral      = self._integral + self.e * self._dt

        self.P              = self.Kp * self.e
        self.D              = self.Kd * self._derivative
        self.I              = self.Ki * self._integral

        if self.bclip:
            return np.clip(self.P + self.I + self.D + self._uC,a_min=self._min,a_max=self._max)
        else:
            return self.P + self.I + self.D + self._uC


    def set_target(self,state):
        """ target state for the PID to reach

            Args:
                state : np.array(D)

        """
        self.target     = state
        self.integrator = 0.0
        self.derivator  = 0.0
