""" Models for PID controller """
import numpy as np

class GravityModel:

    def __init__(self,m,g=9.81):
        self._m = m
        self._g = g
        self.u  = np.array([0,0,m * g],dtype=float)

    def get_control(self):
        return self.u
