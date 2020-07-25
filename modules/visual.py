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


class Animator:
   def __init__(self, plt, total_iters, interval_ms,
                ideal_data_f, meas_data_f, k_filter):
      self.plt = plt
      self.plt.style.use('ggplot')
      # self.fig = plt.figure()
      self.fig = plt.figure(constrained_layout=True)
      # self.ax1 = self.fig.add_subplot(1, 1, 1)
      gs = self.fig.add_gridspec(3, 4)
      self.ax1 = self.fig.add_subplot(gs[:3, :3])
      self.ax2 = self.fig.add_subplot(gs[0, 3])
      self.ax3 = self.fig.add_subplot(gs[1, 3])
      self.ax4 = self.fig.add_subplot(gs[2, 3])
      
      self.idata_f = ideal_data_f
      self.mdata_f = meas_data_f
      self.kf = k_filter

      self.iters = total_iters
      self.interval = interval_ms
      
      self.line_i, = self.ax1.plot([], [], 'g-', alpha=0.3, label='ideal')
      self.line_m, = self.ax1.plot([], [], 'b-', alpha=0.3, label='measured')
      self.line_p, = self.ax1.plot([], [], 'ro', label='predicted')

      self.line_iax, = self.ax2.plot([], [], 'g-', label='ideal')
      self.line_max, = self.ax2.plot([], [], 'b-', label='measured')
      self.line_pax, = self.ax2.plot([], [], 'r-', label='predicted')

      self.line_iay, = self.ax3.plot([], [], 'g-', label='ideal')
      self.line_may, = self.ax3.plot([], [], 'b-', label='measured')
      self.line_pay, = self.ax3.plot([], [], 'r-', label='predicted')

      self.line_iaq, = self.ax4.plot([], [], 'g-', label='ideal')
      self.line_maq, = self.ax4.plot([], [], 'b-', label='measured')
      self.line_paq, = self.ax4.plot([], [], 'r-', label='predicted')

      self.ax1.set_xlim(-5, 105)
      self.ax1.set_ylim(-5, 65)
      self.ax1.set_title('Robot position (Kalman Filter with sensor fusion gps + imu)')
      self.ax1.set_xlabel('x-coordinate')
      self.ax1.set_ylabel('y-coordinate')
      self.ax1.legend()

      self.ax2.set_xlim(-1, 101)
      self.ax2.set_ylim(-8, 8)
      self.ax2.set_title('Acceleration v/s Time in x')
      self.ax2.set_xlabel('time')
      self.ax2.set_ylabel('acceleration(x)')
      self.ax2.legend()

      self.ax3.set_xlim(-1, 101)
      self.ax3.set_ylim(-8, 8)
      self.ax3.set_title('Acceleration v/s Time in y')
      self.ax3.set_xlabel('time')
      self.ax3.set_ylabel('acceleration(y)')
      self.ax3.legend()

      self.ax4.set_xlim(-1, 101)
      self.ax4.set_ylim(-0.5, 0.5)
      self.ax4.set_title('Acceleration v/s Time in theta')
      self.ax4.set_xlabel('time')
      self.ax4.set_ylabel('acceleration(theta)')
      self.ax4.legend()


      self.i_data = next(self.idata_f)
      self.m_data = next(self.mdata_f)
      self.p_data = self.kf.get_state()
      self.t_data = [0]

      self.ax1.add_patch(self.plot_arrow(self.i_data))
      self.ax1.add_patch(self.plot_arrow(self.m_data))
      self.ax1.add_patch(self.plot_arrow(self.p_data))
   
   def plot_arrow(self, pose, width=0.1,
                  color='black', alpha=0.2):
         return self.ax1.arrow(pose[0], pose[1],
                               np.cos(pose[2]),
                               np.sin(pose[2]),
                               width=width,
                               color=color,
                               alpha=alpha)
   
   def animate(self, i):
      m_dat = next(self.mdata_f)

      self.i_data = np.vstack((self.i_data, next(self.idata_f)))
      self.m_data = np.vstack((self.m_data, m_dat))
      self.p_data = np.vstack((self.p_data, 
                               self.kf.step_update(m_dat, i+1)))

      self.line_i.set_xdata(self.i_data[:i+1, 0])
      self.line_i.set_ydata(self.i_data[:i+1, 1])
      self.line_m.set_xdata(self.m_data[:i+1, 0])
      self.line_m.set_ydata(self.m_data[:i+1, 1])
      self.line_p.set_xdata(self.p_data[:i+1, 0])
      self.line_p.set_ydata(self.p_data[:i+1, 1])

      self.ax1.add_patch(self.plot_arrow(self.i_data[i, :],
                                        color='g'))
      self.ax1.add_patch(self.plot_arrow(self.m_data[i, :],
                                        color='b'))
      self.ax1.add_patch(self.plot_arrow(self.p_data[i, :],
                                        alpha=1))
      self.t_data.append(i+1)

      self.line_iax.set_data(self.t_data[:i+1], self.i_data[:i+1, 3])
      self.line_max.set_data(self.t_data[:i+1], self.m_data[:i+1, 3])
      self.line_pax.set_data(self.t_data[:i+1], self.p_data[:i+1, 3])

      self.line_iay.set_data(self.t_data[:i+1], self.i_data[:i+1, 4])
      self.line_may.set_data(self.t_data[:i+1], self.m_data[:i+1, 4])
      self.line_pay.set_data(self.t_data[:i+1], self.p_data[:i+1, 4])

      self.line_iaq.set_data(self.t_data[:i+1], self.i_data[:i+1, 5])
      self.line_maq.set_data(self.t_data[:i+1], self.m_data[:i+1, 5])
      self.line_paq.set_data(self.t_data[:i+1], self.p_data[:i+1, 5])

      return (self.line_i, self.line_m, self.line_p,
             self.line_iax, self.line_max, self.line_pax, 
             self.line_iay, self.line_may, self.line_pay,
             self.line_iaq, self.line_maq, self.line_paq)

   def run(self):
      anim = FuncAnimation(self.fig, self.animate,
                           frames=self.iters,
                           interval=self.interval,
                           blit=False, repeat=False)
      self.plt.show()



