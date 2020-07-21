# Kalman Filter

A 2D kalman filter tutorial for self<sub>t+dt</sub> and a stanger from space time.

To design a kalman filter for an object moving on a 2D plane with 3 degrees of freedom (traslation in x, traslation in y and rotation along z or yaw).

## The motion model

The following [equations of motion](https://en.wikipedia.org/wiki/Equations_of_motion) describe the motion of the object with acceptable accuracy (neglecting the higher order differentials of the approximation like jerk (literally!) i.e. the derivative of acceleration).

the position of the object in unit time (t+1) given the velocity and acceleration can be predicted as...

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;\newline&space;x_{t&plus;1}&space;=&space;x_t&space;&plus;&space;t\dot{x}_t&space;&plus;&space;\frac{t^2}{2}\ddot{x}_t&space;\\&space;\newline&space;y_{t&plus;1}&space;=&space;y_t&space;&plus;&space;t\dot{y}_t&space;&plus;&space;\frac{t^2}{2}\ddot{y}_t&space;\\&space;\newline&space;\theta_{t&plus;1}&space;=&space;\theta_t&space;&plus;&space;t\dot{\theta}_t&space;&plus;&space;\frac{t^2}{2}\ddot{\theta}_t&space;\\&space;\newline&space;\dot{x}_t&space;=&space;\frac{dx_t}{dt}&space;=&space;\text{Object&space;velocity&space;along&space;x&space;at&space;time&space;t}&space;\\&space;\newline&space;\ddot{x}_t&space;=&space;\frac{d^2x_t}{dt^2}&space;=&space;\text{Object&space;acceleration&space;along&space;x&space;at&space;time&space;t}&space;\\" title="\large \newline x_{t+1} = x_t + t\dot{x}_t + \frac{t^2}{2}\ddot{x}_t \\ \newline y_{t+1} = y_t + t\dot{y}_t + \frac{t^2}{2}\ddot{y}_t \\ \newline \theta_{t+1} = \theta_t + t\dot{\theta}_t + \frac{t^2}{2}\ddot{\theta}_t \\ \newline \dot{x}_t = \frac{dx_t}{dt} = \text{Object velocity along x at time t} \\ \newline \ddot{x}_t = \frac{d^2x_t}{dt^2} = \text{Object acceleration along x at time t} \\" />

</center>

the velocity can be estimated as...

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;\newline&space;\dot{x}_{t&plus;1}&space;=&space;\dot{x}_t&space;&plus;&space;t\ddot{x}_{t}&space;\\&space;\newline&space;\dot{y}_{t&plus;1}&space;=&space;\dot{y}_t&space;&plus;&space;t\ddot{y}_{t}&space;\\&space;\newline&space;\dot{\theta}_{t&plus;1}&space;=&space;\dot{\theta}_t&space;&plus;&space;t\ddot{\theta}_{t}&space;\\" title="\large \newline \dot{x}_{t+1} = \dot{x}_t + t\ddot{x}_{t} \\ \newline \dot{y}_{t+1} = \dot{y}_t + t\ddot{y}_{t} \\ \newline \dot{\theta}_{t+1} = \dot{\theta}_t + t\ddot{\theta}_{t} \\" />

</center>


## State space model

The above motion equations can be represented as a state space equation as. Here our system state is the position (x, y, orientation or heading angle 0 if heading parallel to x axis) and velocity (linear along x and y, angular along z)

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;X_{n&plus;1}&space;=&space;\begin{bmatrix}&space;x_{n&plus;1}&space;\\&space;y_{n&plus;1}&space;\\&space;\theta_{n&plus;1}&space;\\&space;\dot{x}_{n&plus;1}&space;\\&space;\dot{y}_{n&plus;1}&space;\\&space;\dot{\theta}_{n&plus;1}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;\begin{bmatrix}&space;x_{n}&space;\\&space;y_{n}&space;\\&space;\theta_{n}&space;\\&space;\dot{x}_{n}&space;\\&space;\dot{y}_{n}&space;\\&space;\dot{\theta}_{n}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;\frac{t^2}{2}&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\frac{t^2}{2}&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\frac{t^2}{2}&space;\\&space;t&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;t&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;t&space;\end{bmatrix}&space;\begin{bmatrix}&space;\ddot{x}_n&space;\\&space;\ddot{y}_n&space;\\&space;\ddot{\theta}_n&space;\end{bmatrix}" title="\large X_{n+1} = \begin{bmatrix} x_{n+1} \\ y_{n+1} \\ \theta_{n+1} \\ \dot{x}_{n+1} \\ \dot{y}_{n+1} \\ \dot{\theta}_{n+1} \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 & t & 0 & 0 \\ 0 & 1 & 0 & 0 & t & 0 \\ 0 & 0 & 1 & 0 & 0 & t \\ 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_{n} \\ y_{n} \\ \theta_{n} \\ \dot{x}_{n} \\ \dot{y}_{n} \\ \dot{\theta}_{n} \end{bmatrix} + \begin{bmatrix} \frac{t^2}{2} & 0 & 0 \\ 0 & \frac{t^2}{2} & 0 \\ 0 & 0 & \frac{t^2}{2} \\ t & 0 & 0 \\ 0 & t & 0 \\ 0 & 0 & t \end{bmatrix} \begin{bmatrix} \ddot{x}_n \\ \ddot{y}_n \\ \ddot{\theta}_n \end{bmatrix}" />

</center>

This space state equation can be represented as...

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;X_{n&plus;1}&space;=&space;AX_n&space;&plus;&space;BU_n&space;&plus;&space;w" title="\large X_{n+1} = AX_n + BU_n + w" />

where

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;A&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;\text{&space;}&space;B&space;=&space;\begin{bmatrix}&space;\frac{t^2}{2}&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\frac{t^2}{2}&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\frac{t^2}{2}&space;\\&space;t&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;t&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;t&space;\end{bmatrix}" title="\large A = \begin{bmatrix} 1 & 0 & 0 & t & 0 & 0 \\ 0 & 1 & 0 & 0 & t & 0 \\ 0 & 0 & 1 & 0 & 0 & t \\ 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \end{bmatrix} \text{ } B = \begin{bmatrix} \frac{t^2}{2} & 0 & 0 \\ 0 & \frac{t^2}{2} & 0 \\ 0 & 0 & \frac{t^2}{2} \\ t & 0 & 0 \\ 0 & t & 0 \\ 0 & 0 & t \end{bmatrix}" />

</center>

the <img src="https://render.githubusercontent.com/render/math?math=w"> represents the uncertainity or noise in the estimate of the system state as we exclude the higher order differentials in the system.

## Process noise

the process noise <img src="https://render.githubusercontent.com/render/math?math=w"> can be modelled as an equation.

<center>
<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;Q&space;=&space;BQ_aB^T" title="\large Q = BQ_aB^T" />

</center>

where <img src="https://render.githubusercontent.com/render/math?math=Q_a"> is the unceratainity or noise in the acceleration given as standard deviation of acceleration.

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;Q_a&space;=&space;\begin{bmatrix}&space;\sigma_{\ddot{x}}^2&space;&&space;\sigma_{\ddot{y}}.\sigma_{\ddot{x}}&space;&&space;\sigma_{\ddot{\theta}}.\sigma_{\ddot{y}}&space;\\&space;\sigma_{\ddot{x}}.\sigma_{\ddot{y}}&space;&&space;\sigma_{\ddot{y}}^2&space;&&space;\sigma_{\ddot{\theta}}.\sigma_{\ddot{y}}&space;\\&space;\sigma_{\ddot{x}}.\sigma_{\ddot{\theta}}&space;&&space;\sigma_{\ddot{y}}.\sigma_{\ddot{\theta}}&space;&&space;\sigma_{\ddot{\theta}}^2&space;\end{bmatrix}" title="\large Q_a = \begin{bmatrix} \sigma_{\ddot{x}}^2 & \sigma_{\ddot{y}}.\sigma_{\ddot{x}} & \sigma_{\ddot{\theta}}.\sigma_{\ddot{y}} \\ \sigma_{\ddot{x}}.\sigma_{\ddot{y}} & \sigma_{\ddot{y}}^2 & \sigma_{\ddot{\theta}}.\sigma_{\ddot{y}} \\ \sigma_{\ddot{x}}.\sigma_{\ddot{\theta}} & \sigma_{\ddot{y}}.\sigma_{\ddot{\theta}} & \sigma_{\ddot{\theta}}^2 \end{bmatrix}" />

</center>

Assuming there is no correlation between the noise in acceleration of x to the noise in acceleration of y or <img src="https://render.githubusercontent.com/render/math?math=\theta"> we can write <img src="https://render.githubusercontent.com/render/math?math=Q_a"> by equating the non diagonal elements to 0.

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;Q_a&space;=&space;\begin{bmatrix}&space;\sigma_{\ddot{x}}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{\ddot{y}}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{\ddot{\theta}}^2&space;\end{bmatrix}" title="\large Q_a = \begin{bmatrix} \sigma_{\ddot{x}}^2 & 0 & 0 \\ 0 & \sigma_{\ddot{y}}^2 & 0 \\ 0 & 0 & \sigma_{\ddot{\theta}}^2 \end{bmatrix}" />

</center>

## Measurement model

here we model our sensors (eg: GPS, imu) to update the state of our system. the meaurement equation can be written as. 
<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;Y_{t&plus;1}&space;=&space;H&space;X_{t&plus;1}&space;&plus;&space;V" title="\large Y_{t+1} = H X_{t+1} + V" />

</center>

The translation matrix <img src="https://render.githubusercontent.com/render/math?math=H"> allows us to convert meausrements coming in from sensors to the form our state model uses to represent the system.

in our case say we have a local gps system that sends us position updates in same units hence the value 1, this value will change if we use different units for measurement or if we have another representation of position like polar instead of cartesian coordiantes.

the 3 rows say that we have 3 meausrements x,y,<img src="https://render.githubusercontent.com/render/math?math=\theta"> coming from our local GPS while the columns represent the elements in our state. Since we dont measure velocity these columns have value 0.

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;H&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;\end{bmatrix}" title="\large H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0\\ 0 & 1 & 0 & 0 & 0 & 0\\ 0 & 0 & 1 & 0 & 0 & 0 \end{bmatrix}" />

</center>

## Measurement noise

Since there are no perfect measurement devices we have to model the noise in the sensors. This noise is the standard deviation or the spread of the value measured by the sensors. Assuming that the measurement noise of the sensors in different dimesnions is uncorrelated, we set our non diagonal values in the measurement noise matrix to zero.

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;\newline&space;R&space;=&space;HH^TV&space;\\&space;\newline&space;V&space;=&space;\begin{bmatrix}&space;\sigma_{\ddot{x}}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{\ddot{y}}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{\ddot{\theta}}^2&space;\end{bmatrix}&space;\\" title="\large \newline R = HH^TV \\ \newline V = \begin{bmatrix} \sigma_{\ddot{x}}^2 & 0 & 0 \\ 0 & \sigma_{\ddot{y}}^2 & 0 \\ 0 & 0 & \sigma_{\ddot{\theta}}^2 \end{bmatrix} \\" />

</center>

hence in our case

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;R&space;=&space;\begin{bmatrix}&space;\sigma_{\ddot{x}}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{\ddot{y}}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{\ddot{\theta}}^2&space;\end{bmatrix}" title="\large R = \begin{bmatrix} \sigma_{\ddot{x}}^2 & 0 & 0 \\ 0 & \sigma_{\ddot{y}}^2 & 0 \\ 0 & 0 & \sigma_{\ddot{\theta}}^2 \end{bmatrix}" />

</center>

## Algorithm

Now that we have modelled our system as well as the measurements coming into it we are ready to plug in our values into the kalman filter.

The algorithm consists of 2 main steps:

### Prediction step

We can predict the state of the system as well as the noise as given below.

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;\newline&space;X_{t&plus;1}&space;=&space;AX_t&space;&plus;&space;BU&space;\\&space;\newline&space;P_{t&plus;1}&space;=&space;AP_tA^{-1}&space;&plus;&space;Q&space;\\" title="\large \newline X_{t+1} = AX_t + BU \\ \newline P_{t+1} = AP_tA^{-1} + Q \\" />

</center>

### Kalman Gain (K)

Now we calculate the kalman gain <img src="https://render.githubusercontent.com/render/math?math=K">. This value simply represents the proportion in which we should use our measurements versus our prediction to estimate our next state.

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;$K&space;=&space;\frac{P_tH}{HP_tH&plus;R}$" title="\large $K = \frac{P_tH}{HP_tH+R}$" />

</center>

The denominator for this equation <img src="https://latex.codecogs.com/gif.latex?HP_tH&plus;R" title="HP_tH+R" /> is also known as the innovation and is denoted as <img src="https://render.githubusercontent.com/render/math?math=I"> hence the equation can be written as.

<center>
  
<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;\newline&space;K&space;=&space;P_tHI^{-1}&space;\\&space;\newline&space;I&space;=&space;HP_tH&plus;R&space;\\" title="\large \newline K = P_tHI^{-1} \\ \newline I = HP_tH+R \\" />

</center>

The general intution for kalman gain is as the equation below. Higher the noise in measurements the less we trust the external measurements w.r.t our predicted state hence lower the gain.

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;$K&space;=&space;\frac{\text{System&space;Noise&space;(P)}}{\text{System&space;Noise&space;(P)}&space;&plus;&space;\text{Measurement&space;Noise&space;(R)}&space;}$" title="\large $K = \frac{\text{System Noise (P)}}{\text{System Noise (P)} + \text{Measurement Noise (R)} }$" />

</center>

### Correction step

Here we update our system state using our predections and measurements.

<center>
  
<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;$X_{t&plus;1}&space;=&space;X_{t&plus;1}&space;&plus;&space;K[Y_{t&plus;1}&space;-&space;HX_{t&plus;1}]$" title="\large $X_{t+1} = X_{t+1} + K[Y_{t+1} - HX_{t+1}]$" />

</center>

All the step above does is to correct the value we had at predict state using the kalman gain value we calculated

Put simply as the `weighted average` of `measurement` and `prediction`.

<center>

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;$X_{corrected}&space;=&space;X_{predicted}&space;&plus;&space;K(X_{measured}&space;-&space;X_{predicted})$" title="\large $X_{corrected} = X_{predicted} + K(X_{measured} - X_{predicted})$" />

OR

<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;$X_{corrected}&space;=&space;K.X_{measured}&space;&plus;&space;(1-K)&space;X_{predicted}$" title="\large $X_{corrected} = K.X_{measured} + (1-K) X_{predicted}$" />

</center>

