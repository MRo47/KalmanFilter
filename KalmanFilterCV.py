"""
Kalman Filter module
this module is a kalman filter implementation for a constant velocity model 
of a robot moving in 2D space with 3 degrees of freedom x, y 
and rotation around z (theta)

the only available measurement is the robot position in unit timestamp
"""

import logging as log
import numpy as np


class KalmanFilter:
    def __init__(self, state_0, p_diag, q_diag, r_diag, dt=1):
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
        # self.U = accel_0.T  # acceleration 0 for constant velocity model
        print('B: \n', self.B)

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
        # value of diagonal elements is 1 since meausred and state values are in same units
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

    def predict(self):
        ''' predict step of kalman filter '''
        ###################prediction stage#############################################

        self.X = self.A * self.X  # predict the new state

        self.P = self.A * self.P * self.A.T + self.Q  # predict the current covariance
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
        self.R = np.diagflat(r_diag) if r_diag is not None else self.R

        ###############Kalman Gain Calculation##########################################
        I = self.H * self.P * self.H.T + self.R  # Innovation

        K = self.P * self.H.T * I.I  # Kalman gain

        ###############Update step######################################################
        # new observation x,y,t(1x3).T = 3x1
        Y = np.matrix(meas).T

        # new state estimate
        self.X = self.X + K * (Y - self.H * self.X)  

        # update the process covariance
        self.P = (np.identity(6) - K * self.H) * \
            self.P  # new covariance matrix

        # return updated state
        return np.array(self.X).flatten()

    def step_update(self, meas, r_diag=None):
        ''' runs predict step and runs update step if a valid measurement is recieved '''

        # keep original noise input if noise not given
        self.R = np.diagflat(r_diag) if r_diag is not None else self.R

        # run predict step on current data
        self.predict()

        # when no measurement input
        if not self.measurement_valid(meas):  
            # return after prediction only
            return np.array(self.X).flatten()
        
        # update and return state
        return self.update(meas, r_diag=None)

    def get_state(self):
        ''' return state as np.array([x, y, t, velocity-x, velocity-y, velocity-t])'''
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
        coeffs=[25, 0.08, 50, 30] ,num=50, dev=pos_dev, acc_dev=acc_dev)
    #plot data
    plt.plot(xyt_true[0], xyt_true[1], color='#a6e4ff')
    dg.plot_data(plt, xyt_true[0], xyt_true[1],
              xyt_true[2], 'True', c=['#a6e4ff', 'grey'])
    plt.plot(xyt_measured[0], xyt_measured[1], color='blue')
    dg.plot_data(plt, xyt_measured[0], xyt_measured[1],
              xyt_measured[2], 'Measured', c=['blue', 'black'])

    # print(xyt_measured[:, 0])
    state_0 = np.matrix([xyt_measured[0][0], xyt_measured[0][1],
                         xyt_measured[0][2], 2, -2, 0.2])
    p_diag = np.matrix([100, 100, 100, 600, 600, 600])
    q_diag = np.matrix(1e-2 * np.ones(3))
    r_diag = pos_dev #meausurement noise

    filter_obj = KalmanFilter(state_0, p_diag, q_diag, r_diag, dt=1)

    op = [filter_obj.get_state()]
    for xyt_m in xyt_measured[:,1:].T: #for all meausrements except first
        op.append(filter_obj.step_update(xyt_m))
        # break
    
    op = np.array(op)

    print('op ', op.shape)
    plt.plot(op[:, 0], op[:, 1])
    dg.plot_data(plt, op[:,0], op[:,1], op[:,2], label='predicted', c=['red', 'black'])

    ax.legend()
    ax.set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
