import unittest
from ode.rotation.rot import *
from ode.rigid_body.forces import *

import numpy as np


class TestsRotation(unittest.TestCase):
    """
        Test point mass simulation under constant acceleration


    """

    def test_rot_xyz(self):

        rot_xyz     = Rot_zyx(np.pi/2.0,np.pi/2.0,np.pi/2.0)
        rot_xyz_2   = np.dot(Rot_z(np.pi/2.0),np.dot(Rot_y(np.pi/2.0),Rot_x(np.pi/2.0)))

        return np.array_equal(rot_xyz,rot_xyz_2)

    def test_rot_2_euler(self):

        Rot = np.identity(3)

        if np.sum(Rot2Euler(Rot)) == 0.0:
            return True
        else:
            return False
