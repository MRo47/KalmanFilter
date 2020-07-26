"""
Kalman Filter module
this module is a kalman filter implementation for a constant velocity model 
of a robot moving in 2D space with 3 degrees of freedom x, y 
and rotation around z (theta)

the only available measurement is the robot position in unit timestamp
"""

import logging as log
import numpy as np


class KalmanFilterCV:
    def __init__(self, state_0, p_diag, q_diag, r_diag, start_time):
        """ Kalman filter model from initial parameters​        
        Args:   
            state_0: shape 1x6 matrix [pos-x, pos-y, pos-t, velocity-x, velocity-y, velocity-t]
            p_diag: shape 1x6 matrix of covariance [cov-x, cov-y, cov-t, cov-dx, cov-dy, cov-dt]
            q_diag: shape 1x3 matrix of acceleration covariance (approx. estimate) [cov-d2x, cov-d2y, cov-d2t]
            r_diag: shape 1x3 matrix of measurement covariance (sensor noise) [cov-x, cov-y, cov-t] 
        
        measurement noise r_diag is used as default, unless noise estimate is available at update for every point
        """
        # state space model
        self.X = state_0.T  # [x, y, t, x', y', t']
        
        # compute A with time interval t
        self.A = lambda t: np.matrix([[1, 0, 0, t, 0, 0],
                                      [0, 1, 0, 0, t, 0],
                                      [0, 0, 1, 0, 0, t],
                                      [0, 0, 0, 1, 0, 0],
                                      [0, 0, 0, 0, 1, 0],
                                      [0, 0, 0, 0, 0, 1]])
        print('A(1): \n', self.A(1))

        self.B = lambda t : np.vstack((0.5*t**2*np.eye(3), t*np.eye(3)))
        # acceleration 0 for constant velocity model
        # also no external control inputs are provided U = 0
        print('B(1): \n', self.B(1))

        # initial variance
        self.P = np.diagflat(p_diag)
        print('P: \n', self.P)

        # Process noise
        self.q_diag = np.diagflat(q_diag)
        print('q_diag: \n', q_diag)

        # Process noise matrix
        print('Q(1): \n', self.Q(1))

        # transform matrix, we measure only positon hence
        # value of diagonal elements is 1 since meausred and state values are in same units
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])

        # measurement noise
        self.R = np.diagflat(r_diag)
        print('R: \n', self.R)

        #last update
        self.last_update = start_time
        print('Start time: ', self.last_update)

    def Q(self, t):
        ''' compute Q with time interval t '''
        # Q could be a lambda function but this would mean 
        # B will be computed twice B and B.T hence the value is taken once here
        # Then transposed
        B_t = self.B(t)
        return B_t * self.q_diag * B_t.T

    def __str__(self):
        """ State of the kalman filter """
        return (f'Kalman Filter \n' +
                f'X (State) :\n {self.X} \n\n' +
                f'A (Dynamics matrix) :\n {self.A(1)} \n\n' +
                f'P (Noise Covariance matrix):\n {self.P} \n\n' +
                f'Q (Process Covariance matrix):\n {self.Q(1)} \n\n' +
                f'H (Translation matrix):\n {self.H} \n\n' +
                f'R (Measurement noise matrix):\n {self.R} \n\n')
    
    def measurement_valid(self, meas):
        ''' check if a measurement is valid '''
        return meas is not None and not np.isnan(meas).any()

    def predict(self, dt):
        """ predict step of kalman filter 
        Args:
            dt: time differnce to last update
        """
        ###################prediction stage#############################################
        if(dt >= 0.0001):  # if updates happen at same time state doesnt change
            # predict the new state
            self.X = self.A(dt) * self.X
            # print('X predicted: \n', self.X)
            # predict the current covariance (system noise only)
            a = self.A(dt)
            self.P = a * self.P * a.T + self.Q(dt)
            # print('P predicted: \n', self.P)
        # print('Process Covariance after predict step:\n', self.P)
        return

    def update(self, meas, noise=None):
        ''' update step of kalman filter 
        Args:
            meas: position measurement, array like of dims 1X3 (x, y, t)
            noise: (optional) measurement covariance of dims 1x3
        '''

        # if no measurement input no update step is performed
        if not self.measurement_valid(meas):
            return

        self.R = np.diagflat(noise) if noise is not None else self.R

        ###############Kalman Gain Calculation##########################################
        I = self.H * self.P * self.H.T + self.R  # Innovation

        K = self.P * self.H.T * I.I  # Kalman gain

        ###############Update step######################################################
        
        # new observation x,y,t(1x3).T = 3x1
        Y = np.matrix(meas).T

        # new state estimate
        self.X = self.X + K * (Y - self.H * self.X)  

        # update the process covariance
        self.P = (np.identity(6) - K * self.H) * self.P

        return

    def step_update(self, meas, time_stamp, noise=None):
        ''' runs predict step and runs update step if a valid measurement is recieved '''
        
        # keep original noise input if noise not given
        noise = np.diagflat(noise) if noise is not None else None

        # predict new state with time diff
        self.predict(time_stamp - self.last_update)

        # update state
        # only use pos measurements for update
        self.update(meas[:3], noise=noise)

        # reset timestamp
        self.last_update = time_stamp

        return np.array(self.X).flatten()

    def get_state(self):
        ''' return state as np.array([x, y, t, velocity-x, velocity-y, velocity-t])'''
        return np.array(self.X).flatten()
    
    def get_noise(self):
        return np.diagonal(self.P)
