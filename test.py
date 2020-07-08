# import numpy as np

# dt = 1

# A = np.eye(6)
# A[[0, 1, 2], [3, 4, 5]] = dt  # discrete time constant

# print(A)

# A = np.eye(6)
# A[([0, 3], [2, 5])] = dt  # discrete time constant

# print(A)

# B = np.vstack((0.5*dt**2*np.eye(3), dt*np.eye(3)))

# print(B)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'ro')


def init():
    ax.set_xlim(0, 2*np.pi)
    ax.set_ylim(-1, 1)
    return ln,


def update(frame):
    xdata.append(frame)
    ydata.append(np.sin(frame))
    ln.set_data(xdata, ydata)
    return ln,


ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
                    init_func=init, blit=True)
plt.show()