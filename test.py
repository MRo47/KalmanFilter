# import numpy as np

# dt = 1

# A = np.eye(6)
# A[[0, 1, 2], [3, 4, 5]] = dt  # discrete time constant

# print(A)

# A = np.eye(6)
# A[([0, 3], [2, 5])] = dt  # discrete time constant

# print(A)

# B = np.vstack((0.5*dt**2*np.eye(3), dt*np.eye(3)))

# print(B)

###############################################################

# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# fig, ax = plt.subplots()
# xdata, ydata = [], []
# ln, = plt.plot([], [], 'ro')


# def init():
#     ax.set_xlim(0, 2*np.pi)
#     ax.set_ylim(-1, 1)
#     return ln,


# def update(frame):
#     xdata.append(frame)
#     ydata.append(np.sin(frame))
#     ln.set_data(xdata, ydata)
#     return ln,


# ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
#                     init_func=init, blit=True)
# plt.show()

###############################################################
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
            tuple: x, y, theta, d2x/d22, d2y/dt2, d2theta/dt2
        """
        for t in np.linspace(self.min_t,
                             self.max_t,
                             self.num):
            # x is linear, will have constant velocity thus 0 acceleration
            yield (self.x(t), self.y(t), self.theta(t),
                   self.dnx_dtn(t, 2),
                   self.dny_dtn(t, 2),
                   self.dntheta_dtn(t, 2))

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

        for i, (x, y, q, x_acc, y_acc, t_acc) in enumerate(self.ideal_data()):

            if i in missing_data:
                i += 1
                yield None, None, None, None, None, None
            else:
                i += 1
                yield (np.random.normal(loc=x, scale=dev[0]),
                       np.random.normal(loc=y, scale=dev[1]),
                       np.random.normal(loc=q, scale=dev[2]),
                       np.random.normal(loc=x_acc, scale=acc_dev[0]),
                       np.random.normal(loc=y_acc, scale=acc_dev[1]),
                       np.random.normal(loc=t_acc, scale=acc_dev[2]))


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
        x, y, q, xa, ya, ta = next(ideal_data_f)
        xn, yn, qn, xa, ya, ta = next(noisy_data_f)

        plot_data(ax1, x, y, q, 'ideal')
        if xn is not None:
            plot_data(ax1, xn, yn, qn, 'noisy', c=['red', 'black'])

    ani = FuncAnimation(fig, animate, interval=animation_interval_ms,
                        frames=range(0, total_iters), repeat=False)
    plt.show()


if __name__ == '__main__':
    main()


