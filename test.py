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

    def d_dx(self, x):
        """ A * B * cos( B * x + C ) """
        return (self.c[0] * self.c[1] *
                np.cos(self.c[1] * x + self.c[2]))

    def d2_d2x(self, x):
        """ - A * B^2 * sin( B * x + C) """
        return - (self.c[0] * self.c[1]**2 *
                  np.cos(self.c[1] * x + self.c[2]))


class GenFunction:
    def __init__(self, coeffs=[25, 0.1, 10, 30], 
                 min_x=0, max_x=100, num=20):
        self.func = Func(coeffs=coeffs)
        self.min_x = min_x
        self.max_x = max_x
        self.num = num

    def ideal_data(self):
        # prev_x = 0
        # prev_y = 0

        for x in np.linspace(self.min_x, 
                             self.max_x, 
                             self.num):
            y = self.func.value(x)
            # t = np.arctan2(y-prev_y, x-prev_x)
            t = np.arctan(self.func.d_dx(x))
            # prev_x = x
            # prev_y = y
            yield x, y, t

    def noisy_data(self, dev=[2, 2, 0.01],
                   acc_dev=[0.1, 0.1, 0.01],
                   missing_data=[]):

        # randomly chosen, keep as the data looks good ;-)
        np.random.seed(seed=1168273)

        for x, y, t in self.ideal_data():
            x_ip = x + np.random.normal(loc=0, scale=dev[0])
            y_ip = y + np.random.normal(loc=0, scale=dev[1])
            t_ip = t + np.random.normal(loc=0, scale=dev[2])

            x_ip_acc = self.func.d2_d2x(
                x_ip) + np.random.normal(loc=0, scale=acc_dev[0])
            y_ip_acc = self.func.d2_d2x(
                y_ip) + np.random.normal(loc=0, scale=acc_dev[1])
            t_ip_acc = self.func.d2_d2x(
                t_ip) + np.random.normal(loc=0, scale=acc_dev[2])
            
            # if i in missing_data:
            #     yield None, None, None
            # else:
            yield x_ip, y_ip, t_ip
                #    np.array((x_ip, y_ip, t_ip)), \
                #    np.array((x_ip_acc, y_ip_acc, t_ip_acc))

gen = GenFunction()

x_ = []
y_ = []
t_ = []

for x, y, t in gen.noisy_data():
    # print(x, y, t)
    x_.append(x)
    y_.append(y)
    t_.append(t)

plot_data(plt, x_, y_, t_, 'ideal')

plt.show()

# print('\n')

# for x, y, t in gen.noisy_data():
#     print(x, y, t)

# def ideal_data(min_x=0, max_x=100, num=20,
#                coeffs=[25, 0.1, 10, 30]):

#     func = Func(coeffs)
#     prev_x = 0
#     prev_y = 0

#     for x in np.linspace(min_x, max_x, num):
#         y = func.value(x)
#         t = np.arctan2(y-prev_y, x-prev_x)
#         prev_x = x
#         prev_y = y
#         yield x, y, t





# def noisy_data(min_x=0, max_x=100, num=20,
#                coeffs=[25, 0.1, 10, 30], 
#                dev=[2, 2, 0.01], 
#                acc_dev=[0.1, 0.1, 0.01],
#                missing_data=[]):
    
#     # randomly chosen, keep as the data looks good ;-)
#     np.random.seed(seed=1168273)

#     func = Func(coeffs)

#     for x, y, t in ideal_data(min_x=min_x, max_x=max_x,
#                               num=num, coeffs=coeffs):
#         x_ip = x + np.random.normal(loc=0, scale=dev[0])
#         y_ip = y + np.random.normal(loc=0, scale=dev[1])
#         t_ip = t + np.random.normal(loc=0, scale=dev[2])

#         x_ip_acc = func.d2_d2x(
#             x_ip) + np.random.normal(loc=0, scale=acc_dev[0])
#         y_ip_acc = func.d2_d2x(
#             y_ip) + np.random.normal(loc=0, scale=acc_dev[1])
#         t_ip_acc = func.d2_d2x(
#             t_ip) + np.random.normal(loc=0, scale=acc_dev[2])

        


