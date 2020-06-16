import numpy as np
import matplotlib.pyplot as plt

def plot_data(plt, x, y, t, label, c=['#a6e4ff', 'grey']):
    x_diff = np.cos(t)
    y_diff = np.sin(t)
    plt.quiver(x, y, x_diff, y_diff, color=c[1], width=0.003)
    plt.scatter(x, y, color=c[0], label=label)

def ideal_data(min_x=0, max_x=100, num=20,
               coeffs=[-0.02, 2, 1]):
    x = np.linspace(min_x, max_x, num)
    y = coeffs[0] * x**2 + coeffs[1] * x + coeffs[2]
    # get diff between now and next point
    x_diff = np.diff(x)
    y_diff = np.diff(y)
    t = np.arctan2(y_diff, x_diff)

    return x, y, t

def get_data(min_x=0, max_x=100, num=20,
             coeffs=[-0.02, 2, 1], dev=[2, 2, 0.01],
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
    x, y, t = ideal_data(min_x=min_x, max_x=max_x, num=num+1,
                         coeffs=coeffs)
                         
    #noisy data
    np.random.seed(seed=1168271) #randomly chosen, keep as the data looks good ;-)
    x_ip = x + np.random.normal(loc=0, scale=dev[0], size=num+1)
    y_ip = y + np.random.normal(loc=0, scale=dev[1], size=num+1)

    t_ip = np.arctan2(np.diff(y_ip), np.diff(x_ip))
    t_ip = t_ip + np.random.normal(loc=0, scale=dev[2], size=num)

    #missing data
    for idx in missing_data:
        x_ip[idx] = np.NaN
        y_ip[idx] = np.NaN
        t_ip[idx] = np.NaN

    return (x[:-1],y[:-1],t), (x_ip[:-1], y_ip[:-1], t_ip)


if __name__ == '__main__':
    fig = plt.figure(figsize=(10, 10))
    plt.style.use("ggplot")
    plt.xticks(np.arange(0, 110, 10))
    plt.yticks(np.arange(0, 110, 10))
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    ax = plt.axes()
    (x,y,t), (x_m, y_m, t_m) = get_data()
    #plot data
    plot_data(plt, x, y, t, 'True', c=['#a6e4ff', 'grey'])
    plot_data(plt, x_m, y_m, t_m, 'Measured', c=['blue', 'black'])

    # print(d4)
    ax.legend()
    ax.set_aspect('equal')
    plt.show()
    # print(d)
