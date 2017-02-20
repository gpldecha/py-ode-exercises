import unittest
import gym
import math
from ode.rigid_body.rigid_body_dynamics import *
from ode.rigid_body.forces import *
from ode.rotation.rot import *

from numpy import linalg as LA
import numpy as np


def simulate(rigid_body,force,torque,num_steps,dt):
    for i in range(0,num_steps):
        rigid_body.update(force=force(i),torque=torque(i),dt=dt)


class TestsConstantAcceleration(unittest.TestCase):
    """
        Test point mass simulation under constant acceleration


    """

    def test_free_fall(self):

        rigid_body      = RigidBodyDynamics()
        rigid_body.m    = 1.0
        dt              = 1.0/1000.0
        t               = 1.0 # time of simulation [s]
        num_steps       = int(t / dt)
        g               = 9.81

        start_x         = np.zeros(3)

        rigid_body.x    = start_x

        Fg              = get_Fg(rigid_body.m,g=g)
        force_func      = lambda t: Fg
        torque_func     = lambda t: np.zeros(3)


        simulate(rigid_body,force_func,torque_func,num_steps,dt)

        end_x           = rigid_body.get_position()

        d = 0.5 * g * t**2

        d_s = LA.norm(end_x - start_x)

        d   = int(d * 100.0) # convert [m] to [cm]
        d_s = int(d_s * 100.0)

        self.assertEqual(d,d_s)


    def test_projection(sef):
        """
            The following kinematic equations should hold true

                o   v = at + v0                         (final velocity)
                o   r = r0 + v0 * t + 0.5 * a * t^2     (final position)

        """

        rigid_body      = RigidBodyDynamics()
        rigid_body.m    = 1.0
        dt              = 1.0/1000.0
        t               = 1.0 # time of simulation [s]
        num_steps       = int(t / dt)
        g               = 9.81

        start_x         = np.zeros(3)
        rot_y           = Rot_y(-np.pi/2.5)
        start_v         = np.dot(rot_y,np.array([1,0,0],dtype=float)) * 10.0 # [m/s]


        # set initial conditions
        rigid_body.set_position(start_x)
        rigid_body.set_velocity(start_v)


        Fg              = get_Fg(rigid_body.m,g=9.81)

        for i in range(0,num_steps):
            rigid_body.update(force=Fg,torque=np.zeros(3),dt=dt)


        end_x           = rigid_body.get_position()
        end_v           = rigid_body.get_velocity()

        v               = lambda t: np.array([0,0,-g]) * t + start_v
        r               = lambda t: start_x + (start_v * t) + (0.5 * np.array([0,0,-g]) * t**2)

        diff_x          = int(LA.norm(r(t) - end_x) * 100)
        diff_v          = int(LA.norm(v(t) - end_v) * 100)

        if (diff_v == 0) and (diff_v == 0):
            return True
        else:
            return False
