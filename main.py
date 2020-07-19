import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from KalmanFilterCV import KalmanFilterCV
from KalmanFilter import KalmanFilter
import visual as vis
import DataGen as dg

# total iterations for the simulation
total_iters = 100
# start and end time of simulation
time_lims = [0,100]
# animation display interval in milliseconds
animation_interval_ms = 100
# standard deviation of gaussian noise to be added in position
pos_dev = [2, 2, 0.05]
# standard deviation of gaussian noise to be added in acceleration
acc_dev = [0.1, 0.1, 0.01]
# timestamps at which data was missing from sensors
missing_data = (10, 11, 12, 30, 31, 33, 34, 50, 51, 52)
# initial noise estimate in state (position, velocity)
p_diag = np.matrix([100, 100, 100, 600, 600, 600])
# noise in acceleration
q_diag = np.matrix(1e-2 * np.ones(3))
# meausurement noise
r_diag = pos_dev

# plotter initialiser
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1 = plt.axes(xlim=(-5, 105), ylim=(-5, 65))

# discrete time step 
dt = (time_lims[1] - time_lims[0]) / float(total_iters)

# the path generator function
gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
                 min_t=time_lims[0], max_t=time_lims[1],
                 num=total_iters)

# gen function for ideal data
ideal_data_f = gen.ideal_data()
# gen function for noisy data
noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev,
                              missing_data=missing_data)

# get the first frames and plot
# x, y, q, xa, ya, qa
i_dat = next(ideal_data_f)
# xn, yn, qn, xn_a, yn_a, qn_a
m_dat = next(noisy_data_f)

# initialise kalman filter
# kf = KalmanFilterCV(np.matrix([m_dat[0], m_dat[1], m_dat[2], 0, 0, 0]),
#                     p_diag, q_diag, r_diag, dt=dt)
kf = KalmanFilter(np.matrix([m_dat[0], m_dat[1], m_dat[2], 0, 0, 0]),
                  np.matrix([m_dat[3:6]]),
                  p_diag, q_diag, r_diag, dt=dt)

# current state of kalman filter = m_data input
p_dat = kf.get_state()[:3]
# plot initial state
vis.plot(ax1, i_dat, m_dat, p_dat)
plt.legend()
# animate function fetches data and updates kalman filter
def animate(i):    
    global ax1, kf
    # get ideal and measurement data
    i_dat = next(ideal_data_f)
    m_dat = next(noisy_data_f)
    # update kalman filter
    # p_dat = kf.step_update(np.matrix(m_dat[:3]))[:3]
    p_dat = kf.step_update(np.matrix(m_dat[:3]), np.matrix(m_dat[3:6]))[:3]
    # plot data
    vis.plot(ax1, i_dat, m_dat, p_dat)

# the animator
ani = FuncAnimation(fig, animate,
                    interval=animation_interval_ms,
                    frames=range(0, total_iters), repeat=False)
plt.title('Kalman filter (Constant velocity model)')
plt.show()
