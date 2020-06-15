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

        self.B = np.vstack((0.5*dt**2*np.eye(3), dt*np.eye(3)))
        self.U = accel_0.T #acceleration
        
        #initial variance
        self.P = np.diagflat(p_diag)

        #Process noise
        q_diag = np.diagflat(q_diag)
        
        #process noise
        self.Q = B * q_diag * B.T

        #transform matrix, we use only position and width, height estimates hence
        self.H46 = np.matrix([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0]])

        #measurement noise
        self.R = np.diagflat(r_diag)

    def __str__(self):
        """ State of the kalman filter """
        return (f'Kalman Filter \n' +
                f'X (State) :\n {self.X} \n\n' +
                f'A (Dynamics matrix) :\n {self.A} \n\n' +
                f'P (Noise Covariance matrix):\n {self.P} \n\n' +
                f'Q (Process Covariance matrix):\n {self.Q} \n\n' +
                f'H (Translation matrix):\n {self.H46} \n\n' +
                f'R (Measurement noise matrix):\n {self.R} \n\n')

    def measurement_valid(self, meas):
        ''' check if measurement (Box) is None or if any value is np.nan '''
        return meas is not None and not np.isnan(meas).any()

    def predict(self):
        ''' predict step of kalman filter '''
        ###################prediction stage#############################################
        #assuming a constant velocity model B*U = 0 as no acceleration
        self.X = self.A * self.X  # predict the new state

        self.P = self.A * self.P * self.A.T + self.Q  # predict the current covariance
        #asssuming no relation in errors
        self.P = np.diag(np.diag(self.P))
        # self.P *= np.eye(len(self.P))
        log.info('Process Covariance after predict step:\n%s', self.P)

        return np.array(self.X[:4]).flatten()

    def update(self, meas, r_diag=None):
        ''' update step of kalman filter '''
        ###############Kalman Gain Calculation##########################################
        I = self.H46 * self.P * self.H46.T + self.R  # Innovation Covariance

        K = self.P * self.H46.T * I.I  # Kalman gain

        ###############Update step######################################################
        # new observation only position , width and height xywh (1x4).T = 4x1
        Y = np.matrix(meas).T

        self.X = self.X + K * (Y - self.H46 * self.X)  # new state estimate
        ##update process covar matrix
        self.P = (np.identity(6) - K * self.H46) * \
            self.P  # new covariance matrix

        return np.array(self.X[:4]).flatten()

    def step_update(self, meas, r_diag=None):
        ''' runs predict step and runs update step if a valid measurement is recieved '''
        if r_diag is not None:  # if measurement noise given
            self.R = np.identity(4) * r_diag.T  # else keep original input

        self.predict()

        if not self.measurement_valid(meas):  # when no measurement input
            # return after prediction only
            return np.array(self.X[:4]).flatten()

        return self.update(meas, r_diag=None)

    def get_box(self):
        ''' return box coordinates np.array([x, y, w, h])'''
        return np.array(self.X[:4]).flatten()  # x, y, w, h

    def get_state(self):
        ''' return box coordinates and velocities np.array([x, y, w, h, x_vel, y_vel])'''
        return np.array(self.X).flatten()

    def get_vel(self):
        ''' return box velocity only np.array([x_vel, y_vel])'''
        return np.array(self.X[-2:]).flatten()


def main():
    ''' test kalman filter '''
    import matplotlib.pyplot as plt
    import datagen as dg

    fig = plt.figure(figsize=(20, 20))
    ax = plt.axes()
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')

    #Data input
    missing_data_ids = [15, 16, 17]
    data = dg.get_box_data(ax, min_x=3, max_x=103, num=20, power=1.4,
                           width=11, height=30, dev=[2, 2, 10, 10],
                           missing_data=missing_data_ids)

    print(data)

    p_diag = np.matrix([2, 2, 5, 5, 16, 16])  # [x, y, x', y'] variance (noise)
    q_diag = np.matrix(1e-6 * np.ones(4))  # process noise
    # [x, y, x', y'] measurement noise
    r_diag = np.matrix([4, 4, 100, 100])

    state_0 = np.matrix([data[0][0], data[0][1], data[0][2], data[0][3],
                         data[0][0], 1.4*data[0][1]])
    KF = Kalman2D(state_0, p_diag, q_diag, r_diag, dt=1)

    X_op = []
    Y_op = []
    W_op = []
    H_op = []
    for xywhv in data[1:]:
        new_state = KF.step_update(xywhv)
        temp_x = new_state.tolist()
        X_op.append(temp_x[0])
        Y_op.append(temp_x[1])
        W_op.append(temp_x[2])
        H_op.append(temp_x[3])

    ax.scatter(X_op, Y_op, c='g', label='Filtered')
    for xi, yi, wi, hi in zip(X_op, Y_op, W_op, H_op):
        dg.draw_box(ax, xi, yi, wi, hi, color='g')
    ax.legend()
    plt.show()


if __name__ == '__main__':
    main()
