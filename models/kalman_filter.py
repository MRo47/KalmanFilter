"""
Kalman Filter module
this module is a kalman filter implementation
of a robot moving in 2D space with 3 degrees of freedom x, y 
and rotation around z (theta)

the available measurements are position and imu data (acceleration)
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
            r_diag: shape 1x3 matrix of measurement covariance (sensor noise) [cov-x, cov-y, cov-t] 
        
        measurement noise r_diag is used as default, unless noise estimate is available at update for every point
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
        
        # initial variance
        self.P = np.diagflat(p_diag)
        print('P: \n', self.P)

        # Process noise
        q_diag = np.diagflat(q_diag)
        print('q_diag: \n', q_diag)
        
        # Process noise matrix
        self.Q = self.B * q_diag * self.B.T
        print('Q: \n', self.Q)

        # transform matrix, we measure only positon hence
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])

        # measurement noise
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

    def predict(self, accel_in, imu_noise=None):
        """ predict step of kalman filter 
        accel_in: 3x1 matrix of acceleration data (imu)
        """
        ###################prediction stage#############################################
        accel_in = np.matrix(accel_in)
        # print('accel in: \n', accel_in)

        # predict the new state
        if self.measurement_valid(accel_in):
            self.X = self.A * self.X + self.B * accel_in.T
            # predict the current covariance (with system + imu noise)
            print('imu_diag: \n', np.matrix(imu_noise * np.identity(3)))
            imu_noise = self.B * np.matrix(imu_noise * np.identity(3)) * self.B.T
            self.P = self.A * self.P * self.A.T + imu_noise
        else:
            self.X = self.A * self.X  # predict the new state
            # predict the current covariance (system noise only)
            self.P = self.A * self.P * self.A.T + self.Q
        
        # print('X predicted: \n', self.X)
        # print('P predicted: \n', self.P)
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
        # if no measurement input no update step is performed
        if not self.measurement_valid(meas):
            # return current state
            return np.array(self.X).flatten()

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

    def step_update(self, meas, accel_in, r_diag=None, imu_noise=None):
        ''' runs predict step and runs update step if a valid measurement is recieved '''

        # keep original noise input if noise not given
        self.R = r_diag * np.identity(3) if r_diag is not None else self.R

        self.predict(accel_in, imu_noise=imu_noise)

        return self.update(meas, r_diag=None)

    def get_state(self):
        ''' return box coordinates and velocities np.array([x, y, t, velocity-x, velocity-y, velocity-t])'''
        return np.array(self.X).flatten()
