#   Investigation of numerical approximation methods for first order ODE
#   [1] http://faculty.olin.edu/bstorey/Notes/DiffEq.pdf
#

#  dy/dt = -y
#      y = Ce^-t   (exact solution)

import numpy as np
import matplotlib.pyplot as plt


t = np.linspace(0,2,20)
y = np.exp(-t)

dydt = np.diff(y) / np.diff(t)

t[1:].shape
dydt.shape

plt.figure()
plt.plot(t[1:],dydt,'--r')
plt.plot(t,y)
plt.show()
plt.hold(True)
