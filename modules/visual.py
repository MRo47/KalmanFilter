import numpy as np
import matplotlib.pyplot as plt

def plot_data(plt, xyq, label, c=['#a6e4ff', 'grey'], width=0.1):
    x_diff = np.cos(xyq[2])
    y_diff = np.sin(xyq[2])
    plt.arrow(xyq[0], xyq[1], x_diff, y_diff, color=c[1], width=width)
    plt.scatter(xyq[0], xyq[1], color=c[0], label=label)


def plot(plt, ideal, measured, predicted,
         ideal_c=[(0.18, 0.72, 0.21, 0.5), (0.3, 0.3, 0.3, 0.5)],
         meas_c=[(0.24, 0.53, 1.0, 0.5), (0.3, 0.3, 0.3, 0.5)],
         pred_c=['red', (0.1, 0.1, 0.1, 0.8)]):
         plot_data(plt, ideal, label='ideal', c=ideal_c)
         if(not np.isnan(measured).any()):
            plot_data(plt, measured, label='measured', c=meas_c)
         if(not np.isnan(predicted).any()):
            plot_data(plt, predicted, label='predicted', c=pred_c)

