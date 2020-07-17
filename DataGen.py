import numpy as np
import matplotlib.pyplot as plt
from scipy.misc import derivative


def plot_data(plt, x, y, t, label, c=['#a6e4ff', 'grey']):
    x_diff = np.cos(t)
    y_diff = np.sin(t)
    return (plt.quiver(x, y, x_diff, y_diff, color=c[1], width=0.003),
            plt.scatter(x, y, color=c[0], label=label))

# the function below can be implemented with sympy
# to understand its inner workings it has been implemented here


class PathFunc:

    """ The Path function class that generates the path that the
    robot will follow in space, for a given point in time (t) 
    this function provides the respective x, y and theta value 
    and also the nth derivative using the central difference 
    formula """

    def __init__(self, coeffs=[25, 0.1, 10, 30]):
        """
        Args:
            coeffs (list[A, B, C, D]): for y function A * sin( B * x + C ) + D (default=[25, 0.1, 10, 30])
        """
        self.c = coeffs

    def x(self, t):
        """ x = t linear x(t) """
        return t

    def dnx_dtn(self, t, n, dt=1):
        """ nth order derivative of x(t) at t with spacing dt"""
        return derivative(self.x, t, n=n, dx=dt)

    def y(self, t):
        """ non linear y(t) """
        return (self.c[0] *
                np.sin(self.c[1]*t + self.c[2]) +
                self.c[3])

    def dny_dtn(self, t, n, dt=1):
        """ nth order derivative of y(t) at t """
        return derivative(self.y, t, n=n, dx=dt)

    def theta(self, t):
        """ theta or the heading of the robot at a given time is the
        arc_tangent(dy/dx) where dy/dx is the slope of the curve y(x)
        here we take the partial derivative w.r.t. t of each curve x,y
        to get arctan((dy/dt)/(dx/dt)) """
        return np.arctan2(self.dny_dtn(t, 1), self.dnx_dtn(t, 1))

    def dntheta_dtn(self, t, n, dt=1):
        """ nth order derivative of theta(t) at t """
        return derivative(self.theta, t, n=n, dx=dt)


class PathGen(PathFunc):
    """ The path generator which provides interface to the
        path function to generate a continous stream of ideal
        or noisy data (sensor measurements with gaussian noise) """

    def __init__(self, coeffs=[25, 0.1, 10, 30],
                 min_t=0, max_t=100, num=20):
        """
        Args:
            coeffs (list[A,B,C,D]): for PathFunc() (default=[25, 0.1, 10, 30])
            min_t (int): start time value (default=0)
            max_t (int): stop time value (default=100)
            num (int): total time values between start and stop (default=20)
        """
        super().__init__(coeffs=coeffs)
        self.min_t = min_t
        self.max_t = max_t
        self.num = num

    def ideal_data(self):
        """ generator for ideal data
        Returns:
            np.matrix: [x, y, theta, d2x/d22, d2y/dt2, d2theta/dt2]
        """
        for t in np.linspace(self.min_t,
                             self.max_t,
                             self.num):
            # x is linear, will have constant velocity thus 0 acceleration
            yield np.array([self.x(t), self.y(t), self.theta(t),
                             self.dnx_dtn(t, 2),
                             self.dny_dtn(t, 2),
                             self.dntheta_dtn(t, 2)])

    def noisy_data(self, dev=[2, 2, 0.01],
                   acc_dev=[0.1, 0.1, 0.01],
                   missing_data=()):
        """ generator for noisy data
        Args:
            dev (list): std deviation of noise to be added
                        to position and orientation in [x,y,theta]
            acc_dev (list): std deviation of noise to be added
                            to acceleration in [acc_x, acc_y, acc_theta] 
        
        Returns:
            tuple: x, y, theta, d2x/d22, d2y/dt2, d2theta/dt2 with noise
        """
        # randomly chosen, keep as the data looks good ;-)
        np.random.seed(seed=1168273)

        for i, i_dat in enumerate(self.ideal_data()):

            if i in missing_data:
                i += 1
                yield None
            else:
                i += 1
                yield np.array([np.random.normal(loc=i_dat[0], scale=dev[0]),
                                 np.random.normal(loc=i_dat[1], scale=dev[1]),
                                 np.random.normal(loc=i_dat[2], scale=dev[2]),
                                 np.random.normal(loc=i_dat[3], scale=acc_dev[0]),
                                 np.random.normal(loc=i_dat[4], scale=acc_dev[1]),
                                 np.random.normal(loc=i_dat[5], scale=acc_dev[2])])

def main():
    total_iters = 100
    animation_interval_ms = 100

    gen = PathGen(num=total_iters)

    noisy_data_f = gen.noisy_data(missing_data=(
        20, 21, 22, 23, 50, 51, 52, 53, 54, 55))
    ideal_data_f = gen.ideal_data()

    from matplotlib.animation import FuncAnimation

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)
    ax1 = plt.axes(xlim=(-5, 105), ylim=(-5, 65))

    def animate(i):
        # x, y, q, xa, ya, ta
        i_dat = next(ideal_data_f)
        # xm, ym, qm, xm, ym, qm
        m_dat = next(noisy_data_f)

        plot_data(ax1, i_dat[0], i_dat[1], i_dat[2], 'ideal')
        if m_dat is not None:
            plot_data(ax1, m_dat[0], m_dat[1], m_dat[2],
                      'noisy', c=['red', 'black'])

    ani = FuncAnimation(fig, animate, interval=animation_interval_ms,
                        frames=range(0, total_iters), repeat=False)
    plt.show()


if __name__ == '__main__':
    main()
