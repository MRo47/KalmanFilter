import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import modules.data_gen as dg
import modules.visual as vis
from modules.visual import Animator
from models.kalman_filter_cv import KalmanFilterCV
from models.kalman_filter_fusion import KalmanFilterFusion

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

###################### Kalman Filter Constant Velocity model ############

# the path generator function
gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
                 min_t=time_lims[0], max_t=time_lims[1],
                 num=total_iters)

# timestamps where data is not available (np.nan) from sensors
missing_pos_data = (10, 11, 12, 30, 31, 32, 33, 34, 50, 51, 52)
# acceleration is not used as measurement (constant velocity, acc=0)
missing_accel_data = ()

# gen function for ideal data
ideal_data_f = gen.ideal_data()
# gen function for noisy data
noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev,
                              missing_pos_data=missing_pos_data,
                              missing_accel_data=missing_accel_data)

# initial noise estimate in state (position, velocity)
p_diag = np.matrix([100, 100, 100, 600, 600, 600])
# noise in acceleration
q_diag = np.matrix(1e-2 * np.ones(3))

kf = KalmanFilterCV(np.matrix([0, 0, 0, 0, 0, 0]),
                    p_diag, q_diag, pos_dev, time_lims[0])

anim = Animator('Robot position (Kalman Filter constant velocity model)',
                plt, total_iters, animation_interval_ms,
                ideal_data_f, noisy_data_f, kf)

anim.run()

anim.error_analysis()

###################### Kalman Filter Single Sensor (Position) #################

# the path generator function
gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
                 min_t=time_lims[0], max_t=time_lims[1],
                 num=total_iters)

# timestamps at which data was missing from sensors
# missing position data (eg: gps)
missing_pos_data = ()
# missing_pos_data = (10, 11, 12, 30, 31, 33, 34, 50, 51, 52)
# missing_pos_data = tuple(i for i in range(20, 100))
# missing acceleration data (eg: imus)
# missing_accel_data = ()
# missing_accel_data = (20, 21, 22, 30, 31, 33, 34)
missing_accel_data = tuple(i for i in range(20, 100))
# initial noise estimate in state (position, velocity)
p_diag = np.matrix([100, 100, 100, 600, 600, 600, 1000, 1000, 1000])
# noise in acceleration
q_diag = np.matrix([1e-1, 1e-1, 5e-1])

# gen function for ideal data
ideal_data_f = gen.ideal_data()
# gen function for noisy data
noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev,
                              missing_pos_data=missing_pos_data,
                              missing_accel_data=missing_accel_data)

# initialise kalman filter (with position+acceleration input)
# all values of state are set to zero as we assume nothing is known
# hence a high initial covariance is set (P)
kf = KalmanFilterFusion(np.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0]),
                        p_diag, q_diag, pos_dev, acc_dev, time_lims[0])

ka = Animator('Robot position (Kalman Filter with sensor fusion gps + imu)',
              plt, total_iters, animation_interval_ms,
              ideal_data_f, noisy_data_f, kf)

ka.run()

ka.error_analysis()

###################### Kalman Filter Sensor Fusion ############################

# the path generator function
gen = dg.PathGen(coeffs=[25, 0.1, 10, 30],
                 min_t=time_lims[0], max_t=time_lims[1],
                 num=total_iters)

# timestamps at which data was missing from sensors
# missing position data (eg: gps)
missing_pos_data = (10, 11, 12, 30, 31, 32, 33, 34, 50, 51, 52)
# missing_pos_data = tuple(i for i in range(20, 100))
# missing acceleration data (eg: imus)
# missing_accel_data = ()
missing_accel_data = (20, 21, 22, 30, 31, 33, 34)
# missing_accel_data = tuple(i for i in range(20,100))
# initial noise estimate in state (position, velocity)
p_diag = np.matrix([100, 100, 100, 600, 600, 600, 1000, 1000, 1000])
# noise in acceleration
q_diag = np.matrix([1e-1, 1e-1, 5e-1])

# gen function for ideal data
ideal_data_f = gen.ideal_data()
# gen function for noisy data
noisy_data_f = gen.noisy_data(dev=pos_dev, acc_dev=acc_dev,
                              missing_pos_data=missing_pos_data,
                              missing_accel_data=missing_accel_data)

# initialise kalman filter (with position+acceleration input)
# all values of state are set to zero as we assume nothing is known
# hence a high initial covariance is set (P)
kf = KalmanFilterFusion(np.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0]),
                        p_diag, q_diag, pos_dev, acc_dev, time_lims[0])

ka = Animator('Robot position (Kalman Filter with sensor fusion gps + imu)',
              plt, total_iters, animation_interval_ms,
              ideal_data_f, noisy_data_f, kf)

ka.run()

ka.error_analysis()

# TODO Dead reckoning
# TODO tune Q
