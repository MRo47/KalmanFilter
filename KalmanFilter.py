"""
Kalman2D module
this module is a kalman filter implementation for a constant velocity model of a box with,
filtering on box dimensions and box center position
"""

import logging as log
import numpy as np


class KalmanFilter:
    def __init__(self, state_0, accel_0, p_diag, q_diag, r_diag, dt=1):
        """ Kalman filter model from initial parameters​        
        Args:   
        state_0: shape 1x6 matrix [pos-x, pos-y, pos-t, velocity-x, velocity-y, velocity-t]
        accel_0: shape 1x3 matrix [accel-x, accel-y, accel-t]
        p_diag: shape 1x6 matrix of covariance [cov-x, cov-y, cov-t, cov-dx, cov-dy, cov-dt]
        q_diag: shape 1x3 matrix of acceleration covariance (approx. estimate) [cov-d2x, cov-d2y, cov-d2t]
        r_diag: shape 1x3 matrix of measurement covariance (sensor noise) [cov-x, cov-y, cov-t] (used as defalut)
        """
        # state space model
        self.X = state_0.T  # [x, y, t, x', y', t']
        self.A = np.eye(6)
        self.A[[0, 1, 2], [3, 4, 5]] = dt  # discrete time constant
        print('A: \n', self.A)

        self.B = np.vstack((0.5*dt**2*np.eye(3), dt*np.eye(3)))
        print('B: \n', self.B)
        self.U = accel_0.T #acceleration
        print('U: \n', self.U)
        
        #initial variance
        self.P = np.diagflat(p_diag)
        print('P: \n', self.P)

        #Process noise
        q_diag = np.diagflat(q_diag)
        print('q_diag: \n', q_diag)
        
        #process noise
        self.Q = self.B * q_diag * self.B.T
        print('Q: \n', self.Q)

        #transform matrix, we measure only positon hence
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])

        #measurement noise
        self.R = np.diagflat(r_diag)
        print('R: \n', self.R)

    def __str__(self):
        """ State of the kalman filter """
        return (f'Kalman Filter \n' +
                f'X (State) :\n {self.X} \n\n' +
                f'A (Dynamics matrix) :\n {self.A} \n\n' +
                f'P (Noise Covariance matrix):\n {self.P} \n\n' +
                f'Q (Process Covariance matrix):\n {self.Q} \n\n' +
                f'H (Translation matrix):\n {self.H} \n\n' +
                f'R (Measurement noise matrix):\n {self.R} \n\n')

    def measurement_valid(self, meas):
        ''' check if a measurement is valid '''
        return meas is not None and not np.isnan(meas).any()

    def predict(self, accel_in):
        ''' predict step of kalman filter 
        accel_in: 3x1 matrix of acceleration
        '''
        ###################prediction stage#############################################
        accel_in = np.matrix(accel_in)
        print('accel in: \n', accel_in)

        self.X = self.A * self.X + self.B * accel_in.T # predict the new state

        print('X predicted: \n', self.X)

        self.P = self.A * self.P * self.A.T + self.Q  # predict the current covariance

        print('P predicted: \n', self.P)
        #asssuming no relation in errors
        # self.P = np.diag(np.diag(self.P))

        log.info('Process Covariance after predict step:\n%s', self.P)

        return np.array(self.X).flatten()

    def update(self, meas, r_diag=None):
        ''' update step of kalman filter 
        args:
        meas: position measurement, array like of dims 1X3 (x, y, t)
        r_diag: (optional) measurement covariance of dims 1x3
        '''
        self.R = r_diag * np.identity(3) if r_diag is not None else self.R

        ###############Kalman Gain Calculation##########################################
        I = self.H * self.P * self.H.T + self.R  # Innovation

        K = self.P * self.H.T * I.I  # Kalman gain

        ###############Update step######################################################
        # new observation only position , width and height xywh (1x4).T = 4x1
        Y = np.matrix(meas).T

        self.X = self.X + K * (Y - self.H * self.X)  # new state estimate
        
        # update the process covariance
        self.P = (np.identity(6) - K * self.H) * self.P  # new covariance matrix

        return np.array(self.X).flatten()

    def step_update(self, meas, accel_in, r_diag=None):
        ''' runs predict step and runs update step if a valid measurement is recieved '''

        # keep original input if noise not given
        self.R = r_diag * np.identity(3) if r_diag is not None else self.R

        self.predict(accel_in)

        if not self.measurement_valid(meas):  # when no measurement input
            # return after prediction only
            return np.array(self.X).flatten()

        return self.update(meas, r_diag=None)

    def get_state(self):
        ''' return box coordinates and velocities np.array([x, y, t, velocity-x, velocity-y, velocity-t])'''
        return np.array(self.X).flatten()


def main():
    ''' test kalman filter '''
    import matplotlib.pyplot as plt
    import DataGen as dg

    fig = plt.figure(figsize=(10, 10))
    plt.style.use("ggplot")
    plt.xticks(np.arange(0, 110, 10))
    plt.yticks(np.arange(0, 110, 10))
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    ax = plt.axes()

    pos_dev = [2, 2, 0.05]
    acc_dev = [0.1, 0.1, 0.01]

    xyt_true, xyt_measured, xyt_accel = dg.get_data(
        coeffs=[25, 0.08, 50, 30], num=50, dev=pos_dev, acc_dev=acc_dev)
    #plot data
    #plot data
    plt.plot(xyt_true[0], xyt_true[1], color='#a6e4ff')
    dg.plot_data(plt, xyt_true[0], xyt_true[1],
                 xyt_true[2], 'True', c=['#a6e4ff', 'grey'])
    plt.plot(xyt_measured[0], xyt_measured[1], color='blue')
    dg.plot_data(plt, xyt_measured[0], xyt_measured[1],
                 xyt_measured[2], 'Measured', c=['blue', 'black'])

    state_0 = np.matrix([xyt_measured[0][0], xyt_measured[0][1],
                         xyt_measured[0][2], 2, -2, 0.2])
    accel_0 = np.matrix(xyt_accel[:, 0])
    p_diag = np.matrix([5, 5, 5, 25, 25, 25])
    q_diag = np.matrix(1e-2 * np.ones(3))
    r_diag = pos_dev  # meausurement noise

    filter_obj = KalmanFilter(state_0, accel_0, p_diag, q_diag, r_diag, dt=1)

    op = [filter_obj.get_state()]
    for xyt_m, xyt_acc in zip(xyt_measured[:, 1:].T, xyt_accel[:, 1:].T):  # for all meausrements except first
        # print('xytm shape ', xyt_m.shape)
        # print('xyt acc ', xyt_acc.shape)
        op_val = filter_obj.step_update(xyt_m, xyt_acc)
        # print('op: ', op_val)
        op.append(op_val)
        # break

    op = np.array(op)

    print('op ', op.shape)
    plt.plot(op[:, 0], op[:, 1])
    dg.plot_data(plt, op[:, 0], op[:, 1], op[:, 2],
                 label='predicted', c=['red', 'black'])
    
    ax.legend()
    ax.set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
