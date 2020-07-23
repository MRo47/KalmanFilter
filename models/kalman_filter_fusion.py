"""
Kalman Filter module
this module is a kalman filter implementation
of a robot moving in 2D space with 3 degrees of freedom x, y 
and rotation around z (theta)

the available measurements are position and imu data (acceleration)
"""

import numpy as np
from enum import Enum

# class MotionModel:
#     def __init__(self, A, Q):
#         """
#         Args:
#             A (function): Function that computes A for time dt
#             Q (function): Function that computes Q for time dt
#         """
#         self.A_ = A
#         self.Q_ = Q

#     def A(self, dt):
#         ''' compute with time interval dt '''
#         return self.A_(dt)

#     def Q(self, dt):
#         ''' compute with time interval dt '''
#         return self.Q_(dt)

# class MeasurementModel:
#     def __init__(self, H, R):
#         """
#         Args:
#             H (np.matrix): H matrix that translates measurement updates
#             R (np.matrix): measurement noise covariance
#         """
#         self.H_ = H
#         self.R_ = R

class MType(Enum):
    POS = 1
    IMU = 2

class KalmanFilterFusion:

    MType = MType

    def __init__(self, state_0, p_diag, q_diag, r_pos, r_imu, start_time):
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
        
        print('A(1): \n', self.A(1))

        self.B = lambda t : np.vstack(( 0.16666667*t**3*np.eye(3),
                                        0.5*t**2*np.eye(3),
                                        t*np.eye(3) ))
        print('B(1): \n', self.B(1))
        
        # initial variance
        self.P = np.diagflat(p_diag)
        print('P: \n', self.P)

        # Process noise
        self.q_diag = np.diagflat(q_diag)
        print('q_diag: \n', self.q_diag)

        # Process noise matrix
        print('Q(1): \n', self.Q(1))

        # transform matrix, we measure only positon hence
        self.H_pos = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 1, 0, 0, 0, 0, 0, 0]])

        self.H_imu = np.matrix([[0, 0, 0, 0, 0, 0, 1, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 1]])

        # measurement noise
        self.R_pos = np.diagflat(r_pos)
        self.R_imu = np.diagflat(r_imu)

        print('R_pos: \n', self.R_pos)
        print('R_imu: \n', self.R_imu)

        # last update
        self.last_update = start_time
        print('Start time: ', self.last_update)

    def A(self, t):
        t2 = 0.5*t**2
        return np.matrix([[1, 0, 0, t, 0, 0, t2,  0,   0],
                          [0, 1, 0, 0, t, 0, 0,  t2,   0],
                          [0, 0, 1, 0, 0, t, 0,   0,  t2],
                          [0, 0, 0, 1, 0, 0, t,   0,   0],
                          [0, 0, 0, 0, 1, 0, 0,   t,   0],
                          [0, 0, 0, 0, 0, 1, 0,   0,   t],
                          [0, 0, 0, 0, 0, 0, 1,   0,   0],
                          [0, 0, 0, 0, 0, 0, 0,   1,   0],
                          [0, 0, 0, 0, 0, 0, 0,   0,   1]])
    
    def Q(self, t):
        ''' compute with time interval t '''
        B_t = self.B(t)
        return B_t * self.q_diag * B_t.T
    
    def __str__(self):
        """ State of the kalman filter """
        return (f'Kalman Filter \n' +
                f'X (State) :\n {self.X} \n\n' +
                f'A (Dynamics matrix) :\n {self.A(1)} \n\n' +
                f'P (Noise Covariance matrix):\n {self.P} \n\n' +
                f'Q (Process Covariance matrix):\n {self.Q(1)} \n\n' +
                f'H (Translation matrix):\n {self.H_pos} \n\n' +
                f'R (Measurement noise matrix):\n {self.R_pos} \n\n')

    def measurement_valid(self, meas):
        ''' check if a measurement is valid '''
        return meas is not None and not np.isnan(meas).any()

    def predict(self, dt):
        """ predict step of kalman filter 
        Args:
            accel_in: 3x1 matrix of acceleration data (imu)
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

    def update(self, m_type, meas, noise=None):
        ''' update step of kalman filter 
        args:
        meas: position measurement, array like of dims 1X3 (x, y, t)
        r_diag: (optional) measurement covariance of dims 1x3
        '''
        
        # if no measurement input no update step is performed
        if not self.measurement_valid(meas):
            # return current state
            return np.array(self.X).flatten()

        H = None
        R = None

        if m_type == MType.IMU:
            H = self.H_imu
            R = noise if noise is not None else self.R_imu
        elif m_type == MType.POS:
            H = self.H_pos
            R = noise if noise is not None else self.R_pos
        else:
            raise ValueError('Invalid measurement type')

        ###############Kalman Gain Calculation##########################################
        
        I = H * self.P * H.T + R  # Innovation

        K = self.P * H.T * I.I  # Kalman gain

        ###############Update step######################################################
        
        Y = np.matrix(meas).T  # new observation

        self.X = self.X + K * (Y - H * self.X)  # new state estimate

        # update the process covariance
        self.P = (np.identity(9) - K * H) * self.P  # new covariance matrix
        return 

    def step_update(self, meas, time_stamp, noise=None):
        ''' runs predict step and runs update step if a valid measurement is recieved '''

        # keep original noise input if noise not given
        noise = noise * np.identity(3) if noise is not None else None

        # predict new state with time diff
        self.predict(time_stamp - self.last_update)
        
        # update/correct the estimate

        #update with position
        m_in = np.matrix(meas[:3])
        if self.measurement_valid(m_in):
            # print('meas: ', m_in)
            # print('pos update')
            self.update(MType.POS, m_in, noise=noise)

        #update with imu
        m_in = np.matrix(meas[3:6])
        if self.measurement_valid(m_in):
            # print('meas: ', m_in)
            # print('imu update')
            self.update(MType.IMU, m_in, noise=noise)

        #reset timestamp
        self.last_update = time_stamp

        return np.array(self.X).flatten()

    def get_state(self):
        ''' return box coordinates and velocities np.array([x, y, t, velocity-x, velocity-y, velocity-t])'''
        return np.array(self.X).flatten()
