import numpy as np
from enum import Enum

# enums to indicate measurement type position or imu
class MType(Enum):
    POS = 1 #position sensor
    IMU = 2 #imu

class KalmanFilterFusion:
    """
    Kalman Filter module with sensor fusion
    this module is a kalman filter implementation
    of a robot moving in 2D space with 3 degrees of freedom x, y 
    and rotation around z (theta)
    the model has sensor fusion capabilities with measurements
    coming from a position sensor (eg. gps) and an IMU (acceleration in x, y 
    and angular acceleration for rotation around z or theta)

    the available measurements are position and imu data (acceleration)
    """

    MType = MType #declare the enum as static for this class

    def __init__(self, state_0, p_diag, q_diag, r_pos, r_imu, start_time):
        """ Kalman filter model from initial parameters​        
        
        Args:   
            
            state_0 (np.matrix): shape 1x9 matrix [pos-x, pos-y, pos-theta, 
                                 velocity-x, velocity-y, velocity-theta, 
                                 accel-x , accel-y, accel-theta]
            
            p_diag (np.matrix): shape 1x6 matrix of covariance 
                                [cov-x, cov-y, cov-t, cov-dx, cov-dy, cov-dt]
            
            q_diag (np.matrix): shape 1x3 matrix of covariance or error in 
                                system estimate for neglecting higher order 
                                differentials of taylor series 
                                (linearisation err.) [cov-x, cov-y, cov-t]
            
            r_pos (np.matrix): shape 1x3 matrix of position measurement
                               covariance (sensor noise) [cov-x, cov-y, cov-t]
            
            r_imu (np.matrix): shape 1x3 matrix of imu measurement
                               covariance (sensor noise) [cov-x, cov-y, cov-t]
        
        measurement noise values r_pos and r_imu is used as default,
        unless noise estimate is available at update for every new measurement
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

    def A(self, t):
        """ compute A with time interval t """
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
        """ compute Q with time interval t """
        B_t = self.B(t)
        return B_t * self.q_diag * B_t.T
    
    def __str__(self):
        """ State of the kalman filter """
        return (f'Kalman Filter \n' +
                f'X (State) :\n {self.X} \n\n' +
                f'A(1) (Dynamics matrix for t=1) :\n {self.A(1)} \n\n' +
                f'P (Noise covariance matrix):\n {self.P} \n\n' +
                f'Q(1) (Process noise covariance matrix for t=1):\n {self.Q(1)} \n\n' +
                f'H (Translation matrix):\n {self.H_pos} \n\n' +
                f'R_pos (Measurement noise matrix (pos)):\n {self.R_pos} \n\n' +
                f'R_imu (Measurement noise matrix (imu)):\n {self.R_imu} \n\n')

    def measurement_valid(self, meas):
        """ returns True if meas is not None and if no element is np.nan"""
        return meas is not None and not np.isnan(meas).any()

    def predict(self, dt):
        """ 
        predict step of kalman filter 

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
            # print('Process Covariance after predict step:\n', self.P)


    def update(self, m_type, meas, noise=None):
        """
        update step of kalman filter 

        Args:

            m_type (MType(Enum)): measurement type POS/IMU

            meas (np.array): shape 1x3 sensor measurement

            noise (np.array): shape 1x3 noise input (optional)
                              (default self.r_pos/self.r_imu)

        Raises:

            ValueError: Invalid measurement type if
                        m_type doesnt match MType input
        """
        
        # if no measurement input no update step is performed
        if not self.measurement_valid(meas):
            return

        H = None
        R = None

        #set H and R matrix based on type of sensor measurement
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
        
        # new observation
        Y = np.matrix(meas).T  

        # new state estimate
        self.X = self.X + K * (Y - H * self.X)  

        # update the process covariance
        self.P = (np.identity(9) - K * H) * self.P

    def step_update(self, meas, time_stamp, noise=None):
        """ 
        runs predict step and runs update step if a 
        valid measurement is recieved 
        
        Args:

            meas (np.array): shape 1x3 sensor measurement

            time_stamp (int): sensor measurement time stamp

            noise (np.array): shape 1x3 noise input (optional)
                              (default self.r_pos/self.r_imu)

        Returns:

            state matrix (np.array(1x9)): state matrix X
        """

        # keep original noise input if noise not given
        noise = np.diagflat(noise) if noise is not None else None

        ######## predict new state with time diff
        self.predict(time_stamp - self.last_update)
        
        ######## update/correct the estimate

        #update with position
        m_in = np.matrix(meas[:3])
        if self.measurement_valid(m_in):
            self.update(MType.POS, m_in, noise=noise)

        #update with imu
        m_in = np.matrix(meas[3:6])
        if self.measurement_valid(m_in):
            self.update(MType.IMU, m_in, noise=noise)

        #reset timestamp
        self.last_update = time_stamp

        return np.array(self.X).flatten()

    def get_state(self):
        """ returns state matrix self.X """
        return np.array(self.X).flatten()
    
    def get_noise(self):
        """ returns diagonal of system noise matrix self.P """
        return np.diagonal(self.P)
