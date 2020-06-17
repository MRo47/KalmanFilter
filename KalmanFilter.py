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

        #transform matrix, we measure only positon hence
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])

        #measurement noise
        self.R = np.diagflat(r_diag)

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
        
        self.X = self.A * self.X + self.B * self.U # predict the new state

        self.P = self.A * self.P * self.A.T + self.Q  # predict the current covariance
        #asssuming no relation in errors
        # self.P = np.diag(np.diag(self.P))

        log.info('Process Covariance after predict step:\n%s', self.P)

        return np.array(self.X[:4]).flatten()

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

    def step_update(self, meas, r_diag=None):
        ''' runs predict step and runs update step if a valid measurement is recieved '''

        # keep original input if noise not given
        self.R = r_diag * np.identity(3) if r_diag is not None else self.R

        self.predict()

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

    pos_dev = [1, 1, 0.01]
    acc_dev = [0.1, 0.1, 0.01]

    (x,y,t), (x_m, y_m, t_m), (x_a, y_a, t_a) = dg.get_data(num=50, dev=pos_dev, acc_dev=acc_dev)
    #plot data
    dg.plot_data(plt, x, y, t, 'True', c=['#a6e4ff', 'grey'])
    dg.plot_data(plt, x_m, y_m, t_m, 'Measured', c=['blue', 'black'])

    state_0 = np.matrix([x_m[0], y_m[0], t_m[0], 2, -2, 0.2])
    accel_0 = np.matrix([1,1,0.1])
    p_diag = np.matrix([5, 5, 5, 25, 25, 25])
    q_diag = np.matrix(1e-6 * np.ones(6))

    # #Data input
    # missing_data_ids = [15, 16, 17]
    # data = dg.get_box_data(ax, min_x=3, max_x=103, num=20, power=1.4,
    #                        width=11, height=30, dev=[2, 2, 10, 10],
    #                        missing_data=missing_data_ids)

    # print(data)

    # p_diag = np.matrix([2, 2, 5, 5, 16, 16])  # [x, y, x', y'] variance (noise)
    # q_diag = np.matrix(1e-6 * np.ones(4))  # process noise
    # # [x, y, x', y'] measurement noise
    # r_diag = np.matrix([4, 4, 100, 100])

    # state_0 = np.matrix([data[0][0], data[0][1], data[0][2], data[0][3],
    #                      data[0][0], 1.4*data[0][1]])
    # KF = Kalman2D(state_0, p_diag, q_diag, r_diag, dt=1)

    # X_op = []
    # Y_op = []
    # W_op = []
    # H_op = []
    # for xywhv in data[1:]:
    #     new_state = KF.step_update(xywhv)
    #     temp_x = new_state.tolist()
    #     X_op.append(temp_x[0])
    #     Y_op.append(temp_x[1])
    #     W_op.append(temp_x[2])
    #     H_op.append(temp_x[3])

    # ax.scatter(X_op, Y_op, c='g', label='Filtered')
    # for xi, yi, wi, hi in zip(X_op, Y_op, W_op, H_op):
    #     dg.draw_box(ax, xi, yi, wi, hi, color='g')
    
    ax.legend()
    ax.set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
