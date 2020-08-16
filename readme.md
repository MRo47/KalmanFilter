# Kalman Filter


A 2D kalman filter tutorial for self<sub>t+dt</sub> and any explorer from space time.

To design a kalman filter for an object moving on a 2D plane with 3 degrees of freedom (traslation in x, traslation in y and rotation along z or yaw).

## Tutorial

find tutorial at https://mro47.github.io/portfolio/work-kalman-filter.html

#### Kalman filter tracking robot position by fusing imu and position measurements
<center>
<img src="images/KF_fusion_anim.gif"/>
</center>

## Dependencies
* For modules
   1. numpy
   2. matplotlib
   3. scipy (misc.derivative)
* For notebooks and animations
   1. ffmpeg
   2. jupyter nb

## Run

Run main file
```bash
python3 main.py
```
Edit these lines to play or save animations default=play
```python
anim.run()
# anim.run(save_path='images/KF_constant_velocity_anim.gif')
anim.error_analysis()
# anim.error_analysis(save_path='images/KF_constant_velocity_err.png')
```

Or play with the Jupyter notebook

## Things to play with

* Change functions in **modules/data_gen.py** under functions x and y to generate different paths for the robot.
* Coefficients can be changed or th function can be replaced fully just make sure the **x y limits** are set properly in the plots later in **main.py** otherwise the graphs would be out of bounds.
  ```python
  #pass limits here check the Animator class in modules/visual.py
  anim = Animator('Robot position (Kalman Filter with sensor fusion gps + imu)', plt, total_iters, animation_interval_ms,ideal_data_f, noisy_data_f, kf, Animator.FilterType.SF,start_time=time_lims[0])
  ```
* Change noise in the acceleration and position data from **main.py** at 
  ```python
  # standard deviation of gaussian noise to be added in position
   pos_dev = [2.2, 2.2, 0.05]
   # standard deviation of gaussian noise to be added in acceleration
   acc_dev = [0.1, 0.1, 0.01]
  ```
* Change missing timestamps in **main.py** at 
  ```python
  # timestamps where data is not available (np.nan) from sensors
   missing_pos_data = (10, 11, 12, 30, 31, 32, 33, 34, 50, 51, 52)
   # acceleration is not used as measurement (constant velocity,acc=0)
   missing_accel_data = ()
  ```
* Change and analyse how initial and system noise covariances affect the output
  ```python
  # initial noise estimate in state (position, velocity)
   p_diag = np.matrix([100, 100, 100, 600, 600, 600, 1000, 1000, 1000])
   # noise in acceleration
   q_diag = np.matrix(1e-2 * np.ones(3))
  ```
