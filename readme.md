# Kalman Filter

A tutorial for $\text{self}_{t+1}$

motion model equations

<center>

$x_{t+1} = x_t + t\dot{x}_t + \frac{t^2}{2}\ddot{x}_t$


$y_{t+1} = y_t + t\dot{y}_t + \frac{t^2}{2}\ddot{y}_t$


$\theta_{t+1} = \theta_t + t\dot{\theta}_t + \frac{t^2}{2}\ddot{\theta}_t$


$\dot{x}_{t+1} = \dot{x}_t + t\ddot{x}_{t}$


$\dot{y}_{t+1} = \dot{y}_t + t\ddot{y}_{t}$


$\dot{\theta}_{t+1} = \dot{\theta}_t + t\ddot{\theta}_{t}$

</center>


state space model

<center>

$
X_{n+1} = 
\begin{bmatrix}
x_{n+1} \\
y_{n+1} \\
\theta_{n+1} \\
\dot{x}_{n+1} \\
\dot{y}_{n+1} \\
\dot{\theta}_{n+1}
\end{bmatrix} =
\begin{bmatrix}
1 & 0 & 0 & t & 0 & 0 \\
0 & 1 & 0 & 0 & t & 0 \\
0 & 0 & 1 & 0 & 0 & t \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
x_{n} \\
y_{n} \\
\theta_{n} \\
\dot{x}_{n} \\
\dot{y}_{n} \\
\dot{\theta}_{n}
\end{bmatrix}
+
\begin{bmatrix}
\frac{t^2}{2} & 0 & 0 \\
0 & \frac{t^2}{2} & 0 \\
0 & 0 & \frac{t^2}{2} \\
t & 0 & 0 \\
0 & t & 0 \\
0 & 0 & t
\end{bmatrix}
\begin{bmatrix}
\ddot{x}_n \\
\ddot{y}_n \\
\ddot{\theta}_n
\end{bmatrix}
$

</center>
in short state space
<center>

$X_{n+1} = AX_n + BU_n + w$

</center>

process noise

$Q = BQ_aB^T$

where $Q_a$ is the unceratainity in acceleration given as std dev of acceleration

<center>

$
Q_a =
\begin{bmatrix}
\sigma_{\ddot{x}}^2 & \sigma_{\ddot{y}}.\sigma_{\ddot{x}} & \sigma_{\ddot{\theta}}.\sigma_{\ddot{y}} \\
\sigma_{\ddot{x}}.\sigma_{\ddot{y}} & \sigma_{\ddot{y}}^2 & \sigma_{\ddot{\theta}}.\sigma_{\ddot{y}} \\
\sigma_{\ddot{x}}.\sigma_{\ddot{\theta}} & \sigma_{\ddot{y}}.\sigma_{\ddot{\theta}} & \sigma_{\ddot{\theta}}^2
\end{bmatrix}
$

</center>

neglecting covariances between accelerations in each parameter

<center>

$
Q_a =
\begin{bmatrix}
\sigma_{\ddot{x}}^2 & 0 & 0 \\
0 & \sigma_{\ddot{y}}^2 & 0 \\
0 & 0 & \sigma_{\ddot{\theta}}^2
\end{bmatrix}
$

</center>

measurement model

<center>

$Y_{n+1} = H X_{n+1} + v$

</center>

the measurement translation matrix

Dimension 1 are measurements, dim2 are states
the value 1 says we have same units as measuremets

<center>

$
H =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0\\
0 & 1 & 0 & 0 & 0 & 0\\
0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}
$

</center>

measurement noise

<center>

$R = HH^TV$

$
R =
\begin{bmatrix}
\sigma_{\ddot{x}}^2 & 0 & 0 \\
0 & \sigma_{\ddot{y}}^2 & 0 \\
0 & 0 & \sigma_{\ddot{\theta}}^2
\end{bmatrix}
$

</center>

Algorithm

<center>

$X_{n+1} = AX_n + BU$

$P_{n+1} = APA^{-1} + Q$

</center>

