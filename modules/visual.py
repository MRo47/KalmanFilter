import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from enum import Enum

class FilterType(Enum):
   CV = 1 #constant velocity
   SF = 2 #sensor fusion

class Animator:
   """
   A simulator for kalman filters that runs the data generators to get data and
   update kalman filter states, plots the robot states and performs
   error analysis on the data
   """

   FilterType = FilterType

   def __init__(self, title, plt, total_iters, interval_ms,
                ideal_data_f, meas_data_f, k_filter,
                filter_type, start_time=0, figsize=(20, 10),
                pos_axis_limits=(-5, 105, -5, 65),
                acc_x_limits=(-1, 101, -2,  2),
                acc_y_limits=(-1, 101, -2,  2),
                acc_q_limits=(-1, 101, -0.2,  0.2)):
      """
      Args:
      
         title (string): Name of the main figure

         plt (matplotlib.pyplot): pyplot object from callers scope

         total_iters (int): total iterations to simulate

         interval_ms (int): animation interval between each frame

         ideal_data_f (generator function): for ideal data

         meas_data_f (generator function): for measured/noisy data

         k_filter (KalmanFilter Object): check models

         filter_type (FilterType(Enum)): enum to specify filter model

         start_time (int, default=0): start time of animation

         figsize (tuple, default=(20,10)): size of matplotlib figure

         axis_limits (tuple): (x_min, x_max, y_min, y_max)
      """
      #set parameters
      self.idata_f = ideal_data_f
      self.mdata_f = meas_data_f
      self.kf = k_filter
      self.f_type = filter_type

      self.iters = total_iters
      self.interval = interval_ms

      self.plt = plt
      self.plt.style.use('ggplot')
      self.fig = plt.figure(constrained_layout=True,
                            figsize=figsize)
      
      if self.f_type == FilterType.CV:
         self.ax1 = self.fig.add_subplot(111)
      elif self.f_type == FilterType.SF:
         # add a 3x4
         gs = self.fig.add_gridspec(3, 4)
         self.ax1 = self.fig.add_subplot(gs[:3, :3])
         self.ax2 = self.fig.add_subplot(gs[0, 3])
         self.ax3 = self.fig.add_subplot(gs[1, 3])
         self.ax4 = self.fig.add_subplot(gs[2, 3])
      else:
         raise AttributeError('Invalid filter type passed, available types are' +
                              'CV (constant velocity) and SF (Sensor Fusion)')
      
      #initialise plotting lines for animation
      self.line_i, = self.ax1.plot([], [], 'g-', alpha=0.3, label='ideal')
      self.line_m, = self.ax1.plot([], [], 'b-', alpha=0.3, label='measured')
      self.line_p, = self.ax1.plot([], [], 'ro', label='predicted')

      if self.f_type == FilterType.SF:
         self.line_iax, = self.ax2.plot([], [], 'g-', label='ideal')
         self.line_max, = self.ax2.plot([], [], 'b-', label='measured')
         self.line_pax, = self.ax2.plot([], [], 'r-', label='predicted')

         self.line_iay, = self.ax3.plot([], [], 'g-', label='ideal')
         self.line_may, = self.ax3.plot([], [], 'b-', label='measured')
         self.line_pay, = self.ax3.plot([], [], 'r-', label='predicted')

         self.line_iaq, = self.ax4.plot([], [], 'g-', label='ideal')
         self.line_maq, = self.ax4.plot([], [], 'b-', label='measured')
         self.line_paq, = self.ax4.plot([], [], 'r-', label='predicted')

      # set axis limits and names
      self.set_axis(self.ax1, pos_axis_limits, title,
                   'x-coordinate', 'y-coordinate')
      
      if self.f_type == FilterType.SF:
         self.set_axis(self.ax2, acc_x_limits, 'Acceleration v/s Time in x',
                       'time', 'acceleration(x)')
         self.set_axis(self.ax3, acc_y_limits, 'Acceleration v/s Time in y',
                       'time', 'acceleration(y)')
         self.set_axis(self.ax4, acc_q_limits, 'Acceleration v/s Time in theta',
                       'time', 'acceleration(theta)')

      # get initial data
      self.i_data = np.array([next(self.idata_f)]) #2d
      self.m_data = np.array([next(self.mdata_f)]) #2d
      # get current state of kalman filter
      self.p_data = np.array([self.kf.get_state()]) #2d
      self.sn_data = self.kf.get_noise()[:3]
      # t_data is time stamps
      self.t_data = [start_time]

      #plot initial data
      self.plot_pos_data(0)

      if self.f_type == FilterType.SF:
         #plot acceleration
         self.plot_acc_data(0)
     
      # animation frame counter
      self.anim_count = start_time
   
   def plot_arrow(self, pose, width=0.1,
                  color='black', alpha=0.2):
       """ plots arrow and returns the artist object """
       self.ax1.add_patch(
            self.ax1.arrow(pose[0], pose[1],
                           np.cos(pose[2]), np.sin(pose[2]),
                           width=width, color=color, alpha=alpha))
   
   def set_axis(self, ax, limits, title, x_label, y_label):
      """ set axis limits and names """
      ax.set_xlim(limits[0], limits[1])
      ax.set_ylim(limits[2], limits[3])
      ax.set_title(title)
      ax.set_xlabel(x_label)
      ax.set_ylabel(y_label)
      ax.legend()
   
   def plot_pos_data(self, t):
      """ set position state data: x, y, theta """
      # print('frame: ', t)
      # set lines
      self.line_i.set_xdata(self.i_data[:t+1, 0])
      self.line_i.set_ydata(self.i_data[:t+1, 1])
      self.line_m.set_xdata(self.m_data[:t+1, 0])
      self.line_m.set_ydata(self.m_data[:t+1, 1])
      self.line_p.set_xdata(self.p_data[:t+1, 0])
      self.line_p.set_ydata(self.p_data[:t+1, 1])
      # set arrows
      self.plot_arrow(self.i_data[t, :], color='g')
      self.plot_arrow(self.m_data[t, :], color='b')
      self.plot_arrow(self.p_data[t, :], alpha=1)
   
   def plot_acc_data(self, t):
      """ set the data on acceleration state graphs """
      # for ideal/measurements acc is on 3,4,5 for x,y,theta
      self.line_iax.set_data(self.t_data[:t+1], self.i_data[:t+1, 3])
      self.line_iay.set_data(self.t_data[:t+1], self.i_data[:t+1, 4])
      self.line_iaq.set_data(self.t_data[:t+1], self.i_data[:t+1, 5])

      self.line_max.set_data(self.t_data[:t+1], self.m_data[:t+1, 3])
      self.line_may.set_data(self.t_data[:t+1], self.m_data[:t+1, 4])
      self.line_maq.set_data(self.t_data[:t+1], self.m_data[:t+1, 5])

      # since SF K filter has 9 state variables acc is on 6,7,8 for x,y,theta
      self.line_pax.set_data(self.t_data[:t+1], self.p_data[:t+1, 6])
      self.line_pay.set_data(self.t_data[:t+1], self.p_data[:t+1, 7])
      self.line_paq.set_data(self.t_data[:t+1], self.p_data[:t+1, 8])
   
   def animate(self, t):
      """
      runs updates on the kalman filter by fetching data from the
      generator functions and plots this on the plots
      """
      self.anim_count+=1
      t=self.anim_count
      if t >= self.iters:
         # have issues when saving figures hence the counter is overidden
         # counts from 0 twice while saving
         return


      # get data
      self.i_data = np.vstack((self.i_data, next(self.idata_f)))
      self.m_data = np.vstack((self.m_data, next(self.mdata_f)))
      # run kalman filter update and get state
      self.p_data = np.vstack((self.p_data, 
                               self.kf.step_update(self.m_data[t, :], t)))
      # get noise inputs
      self.sn_data = np.vstack((self.sn_data,
                                self.kf.get_noise()[:3]))
      self.t_data.append(t)

      # plot the robot positions
      self.plot_pos_data(t)

      if self.f_type == FilterType.SF:
         #plot acceleration
         self.plot_acc_data(t)
         # return artist objects
         return (self.line_i, self.line_m, self.line_p,
                 self.line_iax, self.line_max, self.line_pax,
                 self.line_iay, self.line_may, self.line_pay,
                 self.line_iaq, self.line_maq, self.line_paq)
      else:
         # return artist objects
         return self.line_i, self.line_m, self.line_p

   def run(self, save_path=None, get_anim=False):
      """
      displays, saves or returns animation

      Args:

         get_anim (defalut=False): if True, returns the animation

         save_path (default=None): if None shows the animation else saves to 
                                    save_path given get_anim=False

      returns:

         matplotlib.animation.FuncAnimation or None 
      """
      anim = FuncAnimation(self.fig, self.animate,
                           frames=self.iters-1,
                           interval=self.interval,
                           blit=False, repeat=False)

      if get_anim:
         print('Processing animation.....')
         return anim
      
      if save_path is not None:
         print('Processing animation.....')
         anim.save(save_path, dpi=72, writer='imagemagick')
         print('Saved animation to: ', save_path)
      else:
         self.plt.show()
   
   def rmse(self, preds, targets):
      """ gets root mean squared error """
      return np.sqrt(np.mean(((preds-targets)**2)))
   
   def error_analysis(self, save_path=None):
      """
      computes the root mean square error between ideal and predicted data

      Args:

         save_path (string, default=None): if None displays the figure else
                                           saves at save_path
      """ 
      # root mean square errors
      print('RMSE(x): ',
            self.rmse(self.p_data[:, 0], self.i_data[:, 0]))
      print('RMSE(y): ',
            self.rmse(self.p_data[:, 1], self.i_data[:, 1]))
      print('RMSE(theta): ',
            self.rmse(self.p_data[:, 2], self.i_data[:, 2]))

      # plots for variance
      fig, ax = plt.subplots(3)
      fig.set_size_inches(20, 10)
      fig.suptitle('System noise estimate in pose (P) vs time')

      ax[0].plot(self.t_data, self.sn_data[:, 0],
                 label='P_x', color='red')
      ax[1].plot(self.t_data, self.sn_data[:, 1],
                 label='P_y', color='green')
      ax[2].plot(self.t_data, self.sn_data[:, 2],
                 label='P_theta', color='blue')
      ax[0].set_ylabel('Variance(x)')
      ax[1].set_ylabel('Variance(y)')
      ax[2].set_ylabel('Variance(theta)')
      ax[0].set_xlabel('time')
      ax[1].set_xlabel('time')
      ax[2].set_xlabel('time')
      
      ax[0].set_ylim(-5, 50)
      ax[1].set_ylim(-5, 50)
      ax[2].set_ylim(-5, 50)

      if save_path is not None:
         print('Processing plot.....')
         plt.savefig(save_path, dpi=72, bbox_inches='tight')
         print('Saved plot to: ', save_path)
      else:
         plt.tight_layout()
         plt.show()


# helper functions
def plot_data(ax, xyq, label, c=['#a6e4ff', 'grey'], width=0.1):
    """ plots x, y and theta """
    x_diff = np.cos(xyq[2])
    y_diff = np.sin(xyq[2])
    ax.arrow(xyq[0], xyq[1], x_diff, y_diff, color=c[1], width=width)
    ax.scatter(xyq[0], xyq[1], color=c[0], label=label)


def plot(ax, ideal, measured, predicted,
         ideal_c=[(0.18, 0.72, 0.21, 0.5), (0.3, 0.3, 0.3, 0.5)],
         meas_c=[(0.24, 0.53, 1.0, 0.5), (0.3, 0.3, 0.3, 0.5)],
         pred_c=['red', (0.1, 0.1, 0.1, 0.8)]):
         """ plots x, y, theta for ideal measured and predicted data """
         plot_data(ax, ideal, label='ideal', c=ideal_c)
         if(not np.isnan(measured).any()):
            plot_data(ax, measured, label='measured', c=meas_c)
         if(not np.isnan(predicted).any()):
            plot_data(ax, predicted, label='predicted', c=pred_c)


def plot_acc(ax, time, ideal, measured, predicted,
             ideal_c=[(0.18, 0.72, 0.21, 0.5)],
             meas_c=[(0.24, 0.53, 1.0, 0.5)],
             pred_c='red'):
             """ 
             plots x, y, theta aceleration for ideal,
             measured and predicted data 
             """
             ax.scatter(time, ideal, c=ideal_c)
             if(not np.isnan(measured).any()):
                ax.scatter(time, measured, c=meas_c)
             if(not np.isnan(predicted).any()):
                ax.scatter(time, predicted, c=pred_c)
