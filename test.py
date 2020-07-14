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
    
    def x(self, t):
        """ x = t """
        return t**2
    
    def dnx_dtn(self, t, n):
        """ nth order derivative of x function at t """
        return derivative(self.x , t, n=n)
        
    def y(self, t):
        """ A * sin( B * x + C ) + D """
        return (self.c[0] *
                np.sin(self.c[1]*t + self.c[2]) +
                self.c[3])

    def dny_dtn(self, t, n):
        """ nth order derivative of y function at t """
        return derivative(self.y, t, n=n)

    def theta(self, t):
        """ """
        return np.arctan2(self.dny_dtn(t, 1), self.dnx_dtn(t, 1))

    def dntheta_dtn(self, t, n):
        """ nth order derivative of theta function at x"""
        return derivative(self.theta, t, n=n)


class GenFunction:
    def __init__(self, coeffs=[25, 0.1, 10, 30], 
                 min_t=0, max_t=100, num=20):
        self.func = Func(coeffs=coeffs)
        self.min_t = min_t
        self.max_t = max_t
        self.num = num

    def ideal_data(self):

        for t in np.linspace(self.min_t, 
                             self.max_t, 
                             self.num):
            # x is linear, will have constant velocity thus 0 acceleration
            yield (self.func.x(t), self.func.y(t), self.func.theta(t),
                   self.func.dnx_dtn(t, 2), 
                   self.func.dny_dtn(t, 2), 
                   self.func.dntheta_dtn(t, 2))

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
    x_.append(x)
    y_.append(y)
    t_.append(t)

plt.plot(x_, y_)
# plt.plot(y_)
# plt.plot(t_)
# plot_data(plt, x_, y_, t_, 'ideal')

plt.show()



