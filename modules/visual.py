import numpy as np
import matplotlib.pyplot as plt

def plot_data(ax, xyq, label, c=['#a6e4ff', 'grey'], width=0.1):
    x_diff = np.cos(xyq[2])
    y_diff = np.sin(xyq[2])
    ax.arrow(xyq[0], xyq[1], x_diff, y_diff, color=c[1], width=width)
    ax.scatter(xyq[0], xyq[1], color=c[0], label=label)


def plot(ax, ideal, measured, predicted,
         ideal_c=[(0.18, 0.72, 0.21, 0.5), (0.3, 0.3, 0.3, 0.5)],
         meas_c=[(0.24, 0.53, 1.0, 0.5), (0.3, 0.3, 0.3, 0.5)],
         pred_c=['red', (0.1, 0.1, 0.1, 0.8)]):
         plot_data(ax, ideal, label='ideal', c=ideal_c)
         if(not np.isnan(measured).any()):
            plot_data(ax, measured, label='measured', c=meas_c)
         if(not np.isnan(predicted).any()):
            plot_data(ax, predicted, label='predicted', c=pred_c)

def plot_acc(ax, time, ideal, measured, predicted,
             ideal_c=[(0.18, 0.72, 0.21, 0.5)],
             meas_c=[(0.24, 0.53, 1.0, 0.5)],
             pred_c='red'):
             ax.scatter(time, ideal, c=ideal_c)
             if(not np.isnan(measured).any()):
                ax.scatter(time, measured, c=meas_c)
             if(not np.isnan(predicted).any()):
                ax.scatter(time, predicted, c=pred_c)

from matplotlib.animation import FuncAnimation


class KalmanAnimator:
   def __init__(self, plt, total_iters, interval_ms,
                ideal_data_f, meas_data_f, k_filter):
      self.plt = plt
      self.fig = plt.figure()
      self.kf = k_filter

      self.iters = total_iters
      self.interval = interval_ms
      self.ax = self.fig.add_subplot(1, 1, 1)
      self.line_i, = self.ax.plot([], [], c=(0.18, 0.72, 0.21, 0.5))
      self.line_m, = self.ax.plot([], [], c=(0.24, 0.53, 1.0, 0.5))
      self.line_p, = self.ax.plot([], [], c='red')
      
      self.idata_f = ideal_data_f
      self.mdata_f = meas_data_f

      self.ax.set_xlim(-5, 105)
      self.ax.set_ylim(-5, 65)

      self.i_data = next(self.idata_f)[:3]
      self.m_data = next(self.mdata_f)[:3]
      self.p_data = self.kf.get_state()[:3]

      self.ax.add_patch(self.plot_arrow(self.i_data))
      self.ax.add_patch(self.plot_arrow(self.m_data))
      self.ax.add_patch(self.plot_arrow(self.p_data))
   
   def plot_arrow(self, pose, width=0.1,
                  color=(0.3, 0.3, 0.3, 0.5)):
         return self.plt.arrow(pose[0], pose[1],
                               np.cos(pose[2]),
                               np.sin(pose[2]),
                               width=width,
                               color=color)
   
   def animate(self, i):
      m_dat = next(self.mdata_f)

      self.i_data = np.vstack((self.i_data, next(self.idata_f)[:3]))
      self.m_data = np.vstack((self.m_data, m_dat[:3]))
      self.p_data = np.vstack((self.p_data, 
                               self.kf.step_update(m_dat, i+1)[:3]))

      self.line_i.set_xdata(self.i_data[:i+1, 0])
      self.line_i.set_ydata(self.i_data[:i+1, 1])
      self.line_m.set_xdata(self.m_data[:i+1, 0])
      self.line_m.set_ydata(self.m_data[:i+1, 1])
      self.line_p.set_xdata(self.p_data[:i+1, 0])
      self.line_p.set_ydata(self.p_data[:i+1, 1])

      self.ax.add_patch(self.plot_arrow(self.i_data[i, :]))
      self.ax.add_patch(self.plot_arrow(self.m_data[i, :]))
      self.ax.add_patch(self.plot_arrow(self.p_data[i, :]))

      return self.line_i, self.line_m, self.line_p

   def run(self):
      anim = FuncAnimation(self.fig, self.animate,
                           frames=self.iters,
                           interval=self.interval,
                           blit=False, repeat=False)
      self.plt.show()



