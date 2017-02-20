"""
    Abstract base class for plotting. All objects which are to be plotted
    have to inherit this class.
"""

from abc import ABCMeta, abstractmethod

def is_base_plot(aclass):
    try:
        return aclass.is_base_plot
    except AttributeError:
        return False

class Plot_object(object):
    __metaclass__=ABCMeta

    def __init__(self):
        self.is_base_plot = True

    @abstractmethod
    def draw():
        pass
