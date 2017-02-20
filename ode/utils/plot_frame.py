import numpy as np
from plot_object import *
from ode.rotation.rot import *

class Plot_frame(Plot_object):

    def __init__(self,ax,name=None,scale=1.0):
        super(Plot_frame,self).__init__()

        self.ax     = ax
        self.lw     = 2.0
        self.name   = name
        self.scale  = scale

        self._lines = None
        self._text  = None

        self._pos   = np.zeros(3)
        self._Rot   = np.identity(3)

    def update_pos(self,position):
        self._pos = position

    def update_rot(self,*args):
        """ plots a frame of reference
        """
        if len(args) == 1:
            if args[0].ndim == 1:
                [a,b,g] = args[0]
                self.R  = Rot_zyx(a,b,g)
            else:
                self.R  = args[0]
        elif len(args) == 3:
            self.R      = Rot_zyx(args[0],args[1],args[2])

    def draw(self):
        self._plot_frame(self._pos,self._Rot)


    def _plot_frame(self,pos,Rot):
        """
            Args:
                pos     : np.array(3)       ,  position
                Rot     : np.array((3,3))   ,  orientation
        """

        Rot = Rot * self.scale

        origin = pos
        p1     = origin + Rot[:,0]
        p2     = origin + Rot[:,1]
        p3     = origin + Rot[:,2]

        if self._lines is None:
            line1 = self.ax.plot([origin[0], p1[0]], [origin[1],p1[1]],zs=[origin[2],p1[2]],color='r',linewidth=self.lw)[0]
            line2 = self.ax.plot([origin[0], p2[0]], [origin[1],p2[1]],zs=[origin[2],p2[2]],color='g',linewidth=self.lw)[0]
            line3 = self.ax.plot([origin[0], p3[0]], [origin[1],p3[1]],zs=[origin[2],p3[2]],color='b',linewidth=self.lw)[0]

            self._lines = [line1,line2,line3]

            if self.name is not None:
                self._text = self.ax.text(pos[0], pos[1], pos[2], self.name, color='black')
        else:

            data = np.vstack((origin,p1)).T

            self._lines[0].set_data(data[0:2,:])
            self._lines[0].set_3d_properties(data[2,:])

            data = np.vstack((origin,p2)).T
            self._lines[1].set_data(data[0:2,:])
            self._lines[1].set_3d_properties(data[2,:])

            data = np.vstack((origin,p3)).T
            self._lines[2].set_data(data[0:2,:])
            self._lines[2].set_3d_properties(data[2,:])

            self._text.set_x(origin[0])
            self._text.set_y(origin[1])
            self._text.set_3d_properties(origin[2])
