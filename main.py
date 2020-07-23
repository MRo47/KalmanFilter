import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import modules.data_gen as dg
import modules.visual as vis
from models.kalman_filter_cv import KalmanFilterCV
from models.kalman_filter import KalmanFilter
from models.kalman_filter_fusion import KalmanFilterFusion

###################### Sensors in sync continuous data ##################

# total iterations for the simulation
total_iters = 100
# start and end time of simulation
time_lims = [0,100]
# animation frame display interval in milliseconds
animation_interval_ms = 200
# standard deviation of gaussian noise to be added in position
pos_dev = [2.2, 2.2, 0.05]
# standard deviation of gaussian noise to be added in acceleration
acc_dev = [0.1, 0.1, 0.01]
# initial noise estimate in state (position, velocity)
p_diag = np.matrix([100, 100, 100, 600, 600, 600])
# noise in acceleration
q_diag = np.matrix(1e-2 * np.ones(3))
# meausurement noise
r_diag = pos_dev

###################### Kalman Filter FUSION ############################

# timestamps at which data was missing from sensors
# missing position data (eg: gps)
# missing_pos_data = ()
missing_pos_data = (10, 11, 12, 30, 31, 33, 34, 50, 51, 52)
# missing_pos_data = tuple(i for i in range(20, 100))
# missing acceleration data (eg: imus)
missing_accel_data = ()
# missing_accel_data = (20, 21, 22, 30, 31, 33, 34)
# missing_accel_data = tuple(i for i in range(20,100))
# initial noise estimate in state (position, velocity)
p_diag = np.matrix([100, 100, 100, 600, 600, 600, 1000, 1000, 1000])
# noise in acceleration
q_diag = np.matrix([1e-1, 1e-1, 5e-1])


# the path generator function
gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
                 min_t=time_lims[0], max_t=time_lims[1],
                 num=total_iters)

# gen function for ideal data
ideal_data_f = gen.ideal_data()
# gen function for noisy data
noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev,
                              missing_pos_data=missing_pos_data,
                              missing_accel_data=missing_accel_data)

# get the first frames and plot
# x, y, q, xa, ya, qa
i_dat = next(ideal_data_f)
# xn, yn, qn, xn_a, yn_a, qn_a
m_dat = next(noisy_data_f)

# initialise kalman filter (with acceleration input)
kf = KalmanFilterFusion(np.matrix([m_dat[0], m_dat[1], m_dat[2], 0, 0, 0, 0, 0, 0]),
                        p_diag, q_diag, pos_dev, acc_dev, time_lims[0])

# plotter initialise
plt.style.use('seaborn-darkgrid')
fig, (ax1, ax2, ax3, ax4) = \
    plt.subplots(4, 1, gridspec_kw={'height_ratios': [4, 1, 1, 1]})
ax2.set_ylim(-10, 10)
ax3.set_ylim(-10, 10)
ax4.set_ylim(-0.5, 0.5)

def init():
    # current state of kalman filter = m_data input
    p_dat = kf.get_state()
    # plot initial state
    vis.plot(ax1, i_dat, m_dat, p_dat[:3])
    vis.plot_acc(ax2, 0, i_dat[3], m_dat[3], p_dat[3])
    vis.plot_acc(ax3, 0, i_dat[4], m_dat[4], p_dat[4])
    vis.plot_acc(ax4, 0, i_dat[5], m_dat[5], p_dat[5])
    plt.tight_layout()
    plt.legend()

# animate function fetches data and updates kalman filter

def animate(i):
    # print('\nIter(i): ', i+1)
    # get ideal and measurement data
    i_dat = next(ideal_data_f)
    m_dat = next(noisy_data_f)
    # update kalman filter

    p_dat = kf.step_update(m_dat, i+1)
    vis.plot(ax1, i_dat, m_dat, p_dat[:3])
    vis.plot_acc(ax2, i+1, i_dat[3], m_dat[3], p_dat[3])
    vis.plot_acc(ax3, i+1, i_dat[4], m_dat[4], p_dat[4])
    vis.plot_acc(ax4, i+1, i_dat[5], m_dat[5], p_dat[5])


# the animator
ani = FuncAnimation(fig, animate, init_func=init,
                    interval=animation_interval_ms,
                    frames=range(time_lims[0], time_lims[1]), repeat=False)
plt.title('Kalman filter (sensor fusion)')
plt.show()

# ani.save('images/KF_with_imu_sync.gif', dpi=150, writer='imagemagick')

exit()

###################### Kalman Filter IMU SYNC ############################

# the path generator function
gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
                 min_t=time_lims[0], max_t=time_lims[1],
                 num=total_iters)

# gen function for ideal data
ideal_data_f = gen.ideal_data()
# gen function for noisy data
noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev)

# get the first frames and plot
# x, y, q, xa, ya, qa
i_dat = next(ideal_data_f)
# xn, yn, qn, xn_a, yn_a, qn_a
m_dat = next(noisy_data_f)

# initialise kalman filter (with acceleration input)
kf = KalmanFilter(np.matrix([m_dat[0], m_dat[1], m_dat[2], 0, 0, 0]),
                  np.matrix([m_dat[3:6]]),
                  p_diag, q_diag, r_diag,
                  dt=(time_lims[1] - time_lims[0]) / float(total_iters))

# plotter initialise
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1.axis('equal')
# ax1 = plt.axes(xlim=(-5, 105), ylim=(-5, 105))


def init():
    # current state of kalman filter = m_data input
    p_dat = kf.get_state()[:3]
    # plot initial state
    vis.plot(ax1, i_dat, m_dat, p_dat)
    plt.legend()

# animate function fetches data and updates kalman filter


def animate(i):
    # get ideal and measurement data
    i_dat = next(ideal_data_f)
    m_dat = next(noisy_data_f)
    # update kalman filter
    p_dat = kf.step_update(np.matrix(m_dat[:3]),
                           np.matrix(m_dat[3:6]),
                           imu_noise=acc_dev)[:3]
    # plot data
    vis.plot(ax1, i_dat, m_dat, p_dat)


# the animator
ani = FuncAnimation(fig, animate, init_func=init,
                    interval=animation_interval_ms,
                    frames=range(0, total_iters-1), repeat=False)
plt.title('Kalman filter (with imu input, sync)')
plt.show()

# ani.save('images/KF_with_imu_sync.gif', dpi=150, writer='imagemagick')

###################### Kalman Filter CV SYNC #############################

# # the path generator function
# gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
#                  min_t=time_lims[0], max_t=time_lims[1],
#                  num=total_iters)

# # gen function for ideal data
# ideal_data_f = gen.ideal_data()
# # gen function for noisy data
# noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev)

# # get the first frames and plot
# # x, y, q, xa, ya, qa
# i_dat = next(ideal_data_f)
# # xn, yn, qn, xn_a, yn_a, qn_a
# m_dat = next(noisy_data_f)

# # initialise constant velocity kalman filter (missing acceleration input)
# kf = KalmanFilterCV(np.matrix([m_dat[0], m_dat[1], m_dat[2], 0, 0, 0]),
#                     p_diag, q_diag, r_diag,
#                     dt=(time_lims[1] - time_lims[0]) / float(total_iters))

# # plotter initialise
# fig = plt.figure()
# ax1 = fig.add_subplot(1, 1, 1)
# ax1.axis('equal')
# # ax1 = plt.axes(xlim=(-5, 105), ylim=(-5, 105))

# def init():
#     # current state of kalman filter = m_data input
#     p_dat = kf.get_state()[:3]
#     # plot initial state
#     vis.plot(ax1, i_dat, m_dat, p_dat)
#     plt.legend()

# # animate function fetches data and updates kalman filter
# def animate(i):
#     # get ideal and measurement data
#     i_dat = next(ideal_data_f)
#     m_dat = next(noisy_data_f)
#     # update kalman filter
#     p_dat = kf.step_update(np.matrix(m_dat[:3]))[:3]
#     # plot data
#     vis.plot(ax1, i_dat, m_dat, p_dat)

# # the animator
# ani = FuncAnimation(fig, animate, init_func=init,
#                     interval=animation_interval_ms,
#                     frames=range(0, total_iters-1), repeat=False)
# plt.title('Kalman filter (Constant velocity model, sync)')
# # plt.show()

# ani.save('images/KF_no_imu_sync.gif', dpi=150, writer='imagemagick')

###################### Sensors async ####################################

###################### Kalman Filter CV ASYNC ###########################


# timestamps at which data was missing from sensors
# missing position data (eg: gps)
missing_pos_data = (10, 11, 12, 30, 31, 33, 34, 50, 51, 52)
# missing acceleration data (eg: imus)
missing_accel_data = (20, 21, 22, 30, 31, 33, 34)


# the path generator function
gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
                 min_t=time_lims[0], max_t=time_lims[1],
                 num=total_iters)

# gen function for ideal data
ideal_data_f = gen.ideal_data()
# gen function for noisy data
noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev,
                              missing_pos_data=missing_pos_data,
                              missing_accel_data=missing_accel_data)

# get the first frames and plot
# x, y, q, xa, ya, qa
i_dat = next(ideal_data_f)
# xn, yn, qn, xn_a, yn_a, qn_a
m_dat = next(noisy_data_f)

kf = KalmanFilterCV(np.matrix([m_dat[0], m_dat[1], m_dat[2], 0, 0, 0]),
                    p_diag, q_diag, r_diag,
                    dt=(time_lims[1] - time_lims[0]) / float(total_iters))

# plotter initialise
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1.axis('equal')
# ax1 = plt.axes(xlim=(-5, 105), ylim=(-5, 105))

def init():
    # current state of kalman filter = m_data input
    p_dat = kf.get_state()[:3]
    # plot initial state
    vis.plot(ax1, i_dat, m_dat, p_dat)
    plt.legend()

# animate function fetches data and updates kalman filter
def animate(i):
    # get ideal and measurement data
    i_dat = next(ideal_data_f)
    m_dat = next(noisy_data_f)
    # update kalman filter
    p_dat = kf.step_update(np.matrix(m_dat[:3]))[:3]
    # plot data
    vis.plot(ax1, i_dat, m_dat, p_dat)

# the animator
ani = FuncAnimation(fig, animate, init_func=init,
                    interval=animation_interval_ms,
                    frames=range(0, total_iters-1), repeat=False)
plt.title('Kalman filter (Constant velocity model, async)')
# plt.show()

ani.save('images/KF_no_imu_async.gif', dpi=150, writer='imagemagick')


###################### Kalman Filter IMU ASYNC ##########################

# the path generator function
gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
                 min_t=time_lims[0], max_t=time_lims[1],
                 num=total_iters)

# gen function for ideal data
ideal_data_f = gen.ideal_data()
# gen function for noisy data
noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev,
                              missing_pos_data=missing_pos_data,
                              missing_accel_data=missing_accel_data)

# get the first frames and plot
# x, y, q, xa, ya, qa
i_dat = next(ideal_data_f)
# xn, yn, qn, xn_a, yn_a, qn_a
m_dat = next(noisy_data_f)

# initialise kalman filter (with acceleration input)
kf = KalmanFilter(np.matrix([m_dat[0], m_dat[1], m_dat[2], 0, 0, 0]),
                  np.matrix([m_dat[3:6]]),
                  p_diag, q_diag, r_diag,
                  dt=(time_lims[1] - time_lims[0]) / float(total_iters))

# plotter initialise
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1.axis('equal')
# ax1 = plt.axes(xlim=(-5, 105), ylim=(-5, 105))

def init():
    # current state of kalman filter = m_data input
    p_dat = kf.get_state()[:3]
    # plot initial state
    vis.plot(ax1, i_dat, m_dat, p_dat)
    plt.legend()

# animate function fetches data and updates kalman filter
def animate(i):
    # get ideal and measurement data
    i_dat = next(ideal_data_f)
    m_dat = next(noisy_data_f)
    # update kalman filter
    p_dat = kf.step_update(np.matrix(m_dat[:3]),
                           np.matrix(m_dat[3:6]))[:3]
    # plot data
    vis.plot(ax1, i_dat, m_dat, p_dat)

# the animator
ani = FuncAnimation(fig, animate, init_func=init,
                    interval=animation_interval_ms,
                    frames=range(0, total_iters-1), repeat=False)
plt.title('Kalman filter (with imu input, async)')
# plt.show()

ani.save('images/KF_with_imu_async.gif', dpi=150, writer='imagemagick')
