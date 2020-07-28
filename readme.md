# Kalman Filter

A 2D kalman filter tutorial for self<sub>t+dt</sub> and any explorer from space time.

To design a kalman filter for an object moving on a 2D plane with 3 degrees of freedom (traslation in x, traslation in y and rotation along z or yaw).

Lets start with some color

## Kalman filter tracking robot position



## Motion modelling

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

matrix <img src="https://render.githubusercontent.com/render/math?math=A"> is the system dynamic matrix that aids to predict the next state of the kalman filter given the current state. matrix <img src="https://render.githubusercontent.com/render/math?math=B"> is the control input model and matrix <img src="https://render.githubusercontent.com/render/math?math=U"> is the control vector. If the system has any control inputs they can be modelled in these matrices to predict the next state and accordingly Q should be updated to include the additional noise input (eg: A gas pedal input could be given as a control input to estimate the acceleration in which case the B matrix remains as is, since acceleration is given as input). 

the <img src="https://render.githubusercontent.com/render/math?math=w"> represents the uncertainity or noise in the estimate of the system state as we exclude the higher order differentials in the system.

## Process noise

the process noise <img src="https://render.githubusercontent.com/render/math?math=w"> can be modelled as an equation.

<center>
<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;Q&space;=&space;BQ_aB^T" title="\large Q = BQ_aB^T" />

</center>

where <img src="https://render.githubusercontent.com/render/math?math=Q_a"> is the unceratainity or noise in the acceleration given as standard deviation of acceleration, this is the system noise which is responsible for error in the state prediction. The noise arises as a result of linearisation of system model using the taylor series, here we neglect the higher order terms after acceleration ie: jerk and other high order differentials.

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

<img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;R&space;=&space;HH^TV&space;\newline&space;\newline&space;V&space;=&space;\begin{bmatrix}&space;\sigma_{x}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{y}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{{\theta}}^2&space;\end{bmatrix}" title="R = HH^TV \newline \newline V = \begin{bmatrix} \sigma_{x}^2 & 0 & 0 \\ 0 & \sigma_{y}^2 & 0 \\ 0 & 0 & \sigma_{{\theta}}^2 \end{bmatrix}" />

</center>

hence in our case

<center>

<img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;R&space;=&space;\begin{bmatrix}&space;\sigma_{x}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{y}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{{\theta}}^2&space;\end{bmatrix}" title="R = \begin{bmatrix} \sigma_{x}^2 & 0 & 0 \\ 0 & \sigma_{y}^2 & 0 \\ 0 & 0 & \sigma_{{\theta}}^2 \end{bmatrix}" />

</center>

## Algorithm

Now that we have modelled our system as well as the measurements coming into it we are ready to plug in our values into the kalman filter.

The algorithm consists of 2 main steps: Prediction and Update

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

## Designing Kalman Filters

Designing an application specific kalman filter involves defining the correct matrices based on the chosen state model and measurement (sensors). A good sensor noise model would yeild an accurate filter. 

### Kalman Filter (Constant velocity model)

here we adapt the equations above to model a simple constant velocity model of kalman filter where we neglect acceleration and higher order terms for linearising the system. This filter although very simple would suffice for linear motion as we assume constant velocity motion of a robot. The only measurement given here will be from a position sensor that has some degree of inaccuracy.

The design method flows as fllows

1. Here the same state space model is used as above (position and velocity)

<center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;X_{n}&space;=&space;\begin{bmatrix}&space;x_{n}&space;\\&space;y_{n}&space;\\&space;\theta_{n}&space;\\&space;\dot{x}_{n}&space;\\&space;\dot{y}_{n}&space;\\&space;\dot{\theta}_{n}&space;\end{bmatrix}" title="X_{n} = \begin{bmatrix} x_{n} \\ y_{n} \\ \theta_{n} \\ \dot{x}_{n} \\ \dot{y}_{n} \\ \dot{\theta}_{n} \end{bmatrix}" />
</center>

2. Matrix <img src="https://render.githubusercontent.com/render/math?math=A"> remains the same, matrix <img src="https://render.githubusercontent.com/render/math?math=B"> is not used for prediction since we provide no control input matrix <img src="https://render.githubusercontent.com/render/math?math=U">

<center>
   <img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;A&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;\text{&space;}&space;B&space;=&space;\begin{bmatrix}&space;\frac{t^2}{2}&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\frac{t^2}{2}&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\frac{t^2}{2}&space;\\&space;t&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;t&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;t&space;\end{bmatrix}" title="\large A = \begin{bmatrix} 1 & 0 & 0 & t & 0 & 0 \\ 0 & 1 & 0 & 0 & t & 0 \\ 0 & 0 & 1 & 0 & 0 & t \\ 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \end{bmatrix} \text{ } B = \begin{bmatrix} \frac{t^2}{2} & 0 & 0 \\ 0 & \frac{t^2}{2} & 0 \\ 0 & 0 & \frac{t^2}{2} \\ t & 0 & 0 \\ 0 & t & 0 \\ 0 & 0 & t \end{bmatrix}" />
</center>

3. Process noise matrix <img src="https://render.githubusercontent.com/render/math?math=Q"> in this case will be due to neglecting acceleration and higher order differentials. It remains the same as above.

<center>
<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;Q_a&space;=&space;\begin{bmatrix}&space;\sigma_{\ddot{x}}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{\ddot{y}}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{\ddot{\theta}}^2&space;\end{bmatrix}" title="\large Q_a = \begin{bmatrix} \sigma_{\ddot{x}}^2 & 0 & 0 \\ 0 & \sigma_{\ddot{y}}^2 & 0 \\ 0 & 0 & \sigma_{\ddot{\theta}}^2 \end{bmatrix}" />
</center>

4. Measurement model needs matrix <img src="https://render.githubusercontent.com/render/math?math=H"> which remains the same as above since we have 6 state variables but we measure only 3 variables hence the shape 3x6

<center>
<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{150}&space;\large&space;H&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;\end{bmatrix}" title="\large H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0\\ 0 & 1 & 0 & 0 & 0 & 0\\ 0 & 0 & 1 & 0 & 0 & 0 \end{bmatrix}" />
</center>

5. The measurement noise input to the system will be from the position sensor and is modelled as.

<center>
<img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;R&space;=&space;\begin{bmatrix}&space;\sigma_{x}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{y}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{{\theta}}^2&space;\end{bmatrix}" title="R = \begin{bmatrix} \sigma_{x}^2 & 0 & 0 \\ 0 & \sigma_{y}^2 & 0 \\ 0 & 0 & \sigma_{{\theta}}^2 \end{bmatrix}" />
</center>

6. Once the above matrices have been defined the only work that is left is to plug in the values to the kalman filter algorithm. The prediction step is run and correction is performed when a measurement is available.


### Kalman Filter (Sensor fusion)

Here a kalman filter is designed that estimates the system state based on 2 sensor inputs (position sensor eg: gps and an acceleration sensor eg: imu)

1. The state space model here has 3 more states of the robot (acceleration in x, y and theta) as these variables can be updated using the imu.
   <center>
    <img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;X_{n}&space;=&space;\begin{bmatrix}&space;x_{n}&space;\\&space;y_{n}&space;\\&space;\theta_{n}&space;\\&space;\dot{x}_{n}&space;\\&space;\dot{y}_{n}&space;\\&space;\dot{\theta}_{n}&space;\\&space;\ddot{x}_{n}&space;\\&space;\ddot{y}_{n}&space;\\&space;\ddot{\theta}_{n}&space;\end{bmatrix}" title="X_{n} = \begin{bmatrix} x_{n} \\ y_{n} \\ \theta_{n} \\ \dot{x}_{n} \\ \dot{y}_{n} \\ \dot{\theta}_{n} \\ \ddot{x}_{n} \\ \ddot{y}_{n} \\ \ddot{\theta}_{n} \end{bmatrix}" />
    </center>

2. Matrix <img src="https://render.githubusercontent.com/render/math?math=A"> now has the terms for acceleration from equations of motion, matrix <img src="https://render.githubusercontent.com/render/math?math=B"> has terms for jerk (the differential of acceleration, check taylor series expansion) in this model but is not used for prediction since we provide no control input matrix <img src="https://render.githubusercontent.com/render/math?math=U">. <img src="https://render.githubusercontent.com/render/math?math=B"> will only be used to compute <img src="https://render.githubusercontent.com/render/math?math=Q"> matrix.
   <center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;A&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;&&space;0&space;&&space;\frac{t^2}{2}&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;&&space;0&space;&&space;\frac{t^2}{2}&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;&&space;0&space;&&space;\frac{t^2}{2}&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;t&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\\&space;\end{bmatrix}&space;\text{&space;}&space;B&space;=&space;\begin{bmatrix}&space;\frac{t^3}{6}&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\frac{t^3}{6}&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\frac{t^3}{6}&space;\\&space;\frac{t^2}{2}&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\frac{t^2}{2}&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\frac{t^2}{2}&space;\\&space;t&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;t&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;t&space;\end{bmatrix}" title="A = \begin{bmatrix} 1 & 0 & 0 & t & 0 & 0 & \frac{t^2}{2} & 0 & 0 \\ 0 & 1 & 0 & 0 & t & 0 & 0 & \frac{t^2}{2} & 0 \\ 0 & 0 & 1 & 0 & 0 & t & 0 & 0 & \frac{t^2}{2} \\ 0 & 0 & 0 & 1 & 0 & 0 & t & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 & 0 & t & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & t \\ 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 \\ \end{bmatrix} \text{ } B = \begin{bmatrix} \frac{t^3}{6} & 0 & 0 \\ 0 & \frac{t^3}{6} & 0 \\ 0 & 0 & \frac{t^3}{6} \\ \frac{t^2}{2} & 0 & 0 \\ 0 & \frac{t^2}{2} & 0 \\ 0 & 0 & \frac{t^2}{2} \\ t & 0 & 0 \\ 0 & t & 0 \\ 0 & 0 & t \end{bmatrix}" />
   </center>

3. Process noise matrix <img src="https://render.githubusercontent.com/render/math?math=Q"> in this case will be due to neglecting jerk and other higher order differentials. hence it is given as with the diagonal terms accounting for noise in x, y and theta. The values of diagonals are selected as small values in range 0 to 1.
   <center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;Q_a&space;=&space;\begin{bmatrix}&space;\sigma_{\dddot{x}}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{\dddot{y}}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{\dddot{\theta}}^2&space;\end{bmatrix}" title="Q_a = \begin{bmatrix} \sigma_{\dddot{x}}^2 & 0 & 0 \\ 0 & \sigma_{\dddot{y}}^2 & 0 \\ 0 & 0 & \sigma_{\dddot{\theta}}^2 \end{bmatrix}" />
   </center>

4. Measurement model in this case has 2 sensors which perform the updates 
   
   i) Position sensor model: needs matrix <img src="https://render.githubusercontent.com/render/math?math=H_{pos}"> which has dimensions 3x9 since 3 variables are measured (x, y and theta) and the state has 9 variables.
   <center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;H_{pos}&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;\end{bmatrix}" title="H_{pos} = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\ \end{bmatrix}" />
   </center>

   The noise model for position sensor is given as
   <center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;R_{pos}&space;=&space;\begin{bmatrix}&space;\sigma_{x}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{y}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{{\theta}}^2&space;\end{bmatrix}" title="R_{pos} = \begin{bmatrix} \sigma_{x}^2 & 0 & 0 \\ 0 & \sigma_{y}^2 & 0 \\ 0 & 0 & \sigma_{{\theta}}^2 \end{bmatrix}" />
   </center>

   ii) IMU sensor model: needs matrix <img src="https://render.githubusercontent.com/render/math?math=H_{imu}"> which has dimensions 3x9 since 3 variables are measured, acceleration in x, y and theta and the state has 9 variables.
   <center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;H_{imu}&space;=&space;\begin{bmatrix}&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\\&space;\end{bmatrix}" title="H_{imu} = \begin{bmatrix} 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 \\ \end{bmatrix}" />
   </center>

   The noise model for IMU is given as
   <center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{150}&space;R_{imu}&space;=&space;\begin{bmatrix}&space;\sigma_{\ddot{x}}^2&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;\sigma_{\ddot{y}}^2&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;\sigma_{\ddot{\theta}}^2&space;\end{bmatrix}" title="R_{imu} = \begin{bmatrix} \sigma_{\ddot{x}}^2 & 0 & 0 \\ 0 & \sigma_{\ddot{y}}^2 & 0 \\ 0 & 0 & \sigma_{\ddot{\theta}}^2 \end{bmatrix}" />
   </center>

5. The measurement matrices above <img src="https://render.githubusercontent.com/render/math?math=H_{pos}">, <img src="https://render.githubusercontent.com/render/math?math=R_{pos}"> are used to update the kalman filter when position inputs are avialble to the kalman filter, Matrices <img src="https://render.githubusercontent.com/render/math?math=H_{imu}">, <img src="https://render.githubusercontent.com/render/math?math=R_{imu}"> are used when imu data is available. Given these updates the kalman filter can work asynchronously to update the state when a sensor measurement is available.