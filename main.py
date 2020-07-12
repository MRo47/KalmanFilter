import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import DataGen as dg

def plot_data(plt, x, y, t, label, c=['#a6e4ff', 'grey']):
    x_diff = np.cos(t)
    y_diff = np.sin(t)
    plt.quiver(x, y, x_diff, y_diff, color=c[1], width=0.003)
    plt.scatter(x, y, color=c[0], label=label)

fig, ax = plt.subplots()
x_true, y_true = [], []
x_measured, y_measured = [], []
x_filtered, y_filtered = [], []
ln_true, = plt.plot([], [], 'ro')
ln_measured, = plt.plot([], [], 'ro')

pos_dev = [2, 2, 0.05]
acc_dev = [0.1, 0.1, 0.01]

xyt_true, xyt_measured, xyt_accel = dg.get_data(
    coeffs=[25, 0.08, 50, 30], num=50, dev=pos_dev, acc_dev=acc_dev)

state_0 = np.matrix([xyt_measured[0][0], xyt_measured[0][1],
                     xyt_measured[0][2], 2, -2, 0.2])
p_diag = np.matrix([100, 100, 100, 600, 600, 600])
q_diag = np.matrix(1e-2 * np.ones(3))
r_diag = pos_dev  # meausurement noise
filter_obj = KalmanFilter(state_0, p_diag, q_diag, r_diag, dt=1)

def init():
    ax.set_xlim(-5, 105)
    ax.set_ylim(-5, 65)

    return ln,


def update(frame):
    xdata.append(frame)
    ydata.append(np.sin(frame))
    ln.set_data(xdata, ydata)
    return ln,


ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
                    init_func=init, blit=True)
plt.show()
