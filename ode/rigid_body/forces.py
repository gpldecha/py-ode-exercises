""" A set of standard forces which would act on a rigid body """
from numpy import linalg as LA
import numpy as np

def get_Fg(mass,g):
    """ Gravity force Fd """
    return np.array([0,0,- mass * 9.81],dtype=float)

def get_Fd(v,rho,A,Cd):
    """ Drag force Fd """
    return - 0.5 * rho * A * Cd * LA.norm(v) * v
