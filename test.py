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
    plt.quiver(x, y, x_diff, y_diff, color=c[1], width=0.003)
    plt.scatter(x, y, color=c[0], label=label)

##can be done with symbolics


class Func:
    def __init__(self, coeffs=[25, 0.1, 10, 30]):
        """coeffs: [A, B, C, D]"""
        self.c = coeffs
        
    def value(self, x):
        """ A * sin( B * x + C ) + D """
        return (self.c[0] *
                np.sin(self.c[1]*x + self.c[2]) +
                self.c[3])

    def dy_dx(self, x):
        """ A * B * cos( B * x + C ) """
        return derivative(self.value, x)
        # return (self.c[0] * self.c[1] *
        #         np.cos(self.c[1] * x + self.c[2]))

    def d2y_dx2(self, x):
        """ - A * B^2 * sin( B * x + C) """
        return derivative(self.value, x, n=2)
        # return - (self.c[0] * self.c[1]**2 *
        #           np.cos(self.c[1] * x + self.c[2]))

    def theta(self, x):
        return np.arctan(self.dy_dx(x))

    def dt_dx(self, x):
        return derivative(self.theta, x)
    
    def d2t_dx2(self, x):
        return derivative(self.theta, x, n=2)


class GenFunction:
    def __init__(self, coeffs=[25, 0.1, 10, 30], 
                 min_x=0, max_x=100, num=20):
        self.func = Func(coeffs=coeffs)
        self.min_x = min_x
        self.max_x = max_x
        self.num = num

    def ideal_data(self):

        for x in np.linspace(self.min_x, 
                             self.max_x, 
                             self.num):
            # x is linear, will have constant velocity thus 0 acceleration
            yield (x, self.func.value(x), self.func.theta(x),
                   0, self.func.d2y_dx2(x), self.func.d2t_dx2(x))

    def noisy_data(self, dev=[2, 2, 0.01],
                   acc_dev=[0.1, 0.1, 0.01],
                   missing_data=[]):

        # randomly chosen, keep as the data looks good ;-)
        np.random.seed(seed=1168273)

        for x, y, t, x_acc, y_acc, t_acc in self.ideal_data():        
            yield (np.random.normal(loc=x, scale=dev[0]),
                   np.random.normal(loc=y, scale=dev[1]),
                   np.random.normal(loc=t, scale=dev[2]),
                   np.random.normal(loc=x_acc, scale=acc_dev[0]),
                   np.random.normal(loc=y_acc, scale=acc_dev[1]),
                   np.random.normal(loc=t_acc, scale=acc_dev[2]))
            

gen = GenFunction(num=100)

x_ = []
y_ = []
t_ = []

for x, y, t, xa, ya, ta in gen.ideal_data():
    # print(x, y, t)
    x_.append(xa)
    y_.append(ya)
    t_.append(ta)

plt.plot(t_)
# plot_data(plt, x_, y_, t_, 'ideal')

plt.show()



