import numpy as np
import matplotlib.pyplot as plt

def plot_data(plt, xyq, label, c=['#a6e4ff', 'grey']):
    x_diff = np.cos(xyq[2])
    y_diff = np.sin(xyq[2])
    plt.quiver(xyq[0], xyq[1], x_diff, y_diff, width=0.005, color=c[1])
    plt.scatter(xyq[0], xyq[1], color=c[0], label=label)


def plot(plt, ideal, measured, predicted,
         ideal_c=['#a6e4ff', 'grey'],
         meas_c=['blue', 'grey'],
         pred_c=['red', 'black']):
         plot_data(plt, ideal, label='ideal', c=ideal_c)
         if(measured is not None):
            plot_data(plt, measured, label='measured', c=meas_c)
         if(predicted is not None):
            plot_data(plt, predicted, label='predicted', c=pred_c)