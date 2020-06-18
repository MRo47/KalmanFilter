import numpy as np
import matplotlib.pyplot as plt

def plot_data(plt, x, y, t, label, c=['#a6e4ff', 'grey']):
    x_diff = np.cos(t)
    y_diff = np.sin(t)
    plt.quiver(x, y, x_diff, y_diff, color=c[1], width=0.003)
    plt.scatter(x, y, color=c[0], label=label)

def ideal_data(min_x=0, max_x=100, num=20,
               coeffs=[25, 0.1, 10, 30]):
    x = np.linspace(min_x, max_x, num)
    # y = coeffs[0] * x**2 + coeffs[1] * x + coeffs[2] #[-0.02, 2, 1]
    # y = -0.02 * x**2 + 2 * x + 1
    y = coeffs[0] * np.sin(coeffs[1]*x + coeffs[2]) + coeffs[3]
    # get diff between now and next point
    x_diff = np.diff(x)
    y_diff = np.diff(y)

    t = np.arctan2(y_diff, x_diff)

    print('x diff\n', x_diff)
    print('y diff\n', y_diff)
    print('t diff\n', np.diff(t))

    return x, y, t

def get_data(min_x=0, max_x=100, num=20,
             coeffs=[25, 0.1, 10, 30], dev=[2, 2, 0.01], acc_dev=[0.1, 0.1, 0.01],
             missing_data=[]):
    """
    generate data
    args:
        ax: matplotlib axes
        min_x, max_x: x axis range
        num: total points
        power: the value p in equation y=x^p which defines the path the box moves
        width, height: mean dimensions of the box
        dev: 1D array specifying standard deviation (noise to be added) along [x, y, width, height]
        missing_data: 1D array of indices where data is to be numpy.NaN (no measurement)
    
    returns:
        numpy.array(x, y, w, h) of dimensions num*4
    """
    #true values
    x, y, t = ideal_data(min_x=min_x, max_x=max_x, num=num+2,
                         coeffs=coeffs)
                         
    #noisy data
    np.random.seed(seed=1168271) #randomly chosen, keep as the data looks good ;-)
    x_ip = x + np.random.normal(loc=0, scale=dev[0], size=num+2)
    y_ip = y + np.random.normal(loc=0, scale=dev[1], size=num+2)

    x_ip_acc = np.diff(np.diff(x_ip)) + \
        np.random.normal(loc=0, scale=acc_dev[0], size=num)
    y_ip_acc = np.diff(np.diff(y_ip)) + \
        np.random.normal(loc=0, scale=acc_dev[1], size=num)

    t_ip = t + np.random.normal(loc=0, scale=dev[2], size=num+1)

    t_ip_acc = np.diff(t_ip) + \
        np.random.normal(loc=0, scale=acc_dev[2], size=num)

    print("X acc\n", x_ip_acc)
    print("Y acc\n", y_ip_acc)
    print("T acc\n", t_ip_acc)

    # plt.plot(x[2:], t_ip_acc)
    # plt.show()

    #missing data
    for idx in missing_data:
        x_ip[idx] = np.NaN
        y_ip[idx] = np.NaN
        t_ip[idx] = np.NaN

    print('Data')
    print()

    return np.vstack((x[:-2], y[:-2], t[:-1])), np.vstack((x_ip[:-2], y_ip[:-2], t_ip[:-1])), np.vstack((x_ip_acc, y_ip_acc, t_ip_acc))


if __name__ == '__main__':
    fig = plt.figure(figsize=(10, 10))
    plt.style.use("ggplot")
    plt.xticks(np.arange(0, 110, 10))
    plt.yticks(np.arange(0, 110, 10))
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    ax = plt.axes()
    xyt_true, xyt_measured, xyt_accel = get_data(num=50, dev=[1,1,0.01])

    print(xyt_true[:,1:].shape)
    print(xyt_true[0].shape)
    print(xyt_true[1].shape)
    print(xyt_true[2].shape)

    #plot data
    plot_data(plt, xyt_true[0], xyt_true[1], xyt_true[2], 'True', c=['#a6e4ff', 'grey'])
    plot_data(plt, xyt_measured[0], xyt_measured[1], xyt_measured[2], 'Measured', c=['blue', 'black'])

    
    # print(d4)
    ax.legend()
    ax.set_aspect('equal')
    plt.show()
    # print(d)
