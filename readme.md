# Kalman Filter

A tutorial for $\text{self}_{t+1}$

<center>

$x_{n+1} = x_n + t\dot{x}_n + \frac{t^2}{2}\ddot{x}_n$


$y_{n+1} = y_n + t\dot{y}_n + \frac{t^2}{2}\ddot{y}_n$


$\theta_{n+1} = \theta_n + t\dot{\theta}_n + \frac{t^2}{2}\ddot{\theta}_n$


$\dot{x}_{n+1} = \dot{x}_n + t\ddot{x}_{n}$


$\dot{y}_{n+1} = \dot{y}_n + t\ddot{y}_{n}$


$\dot{\theta}_{n+1} = \dot{\theta}_n + t\ddot{\theta}_{n}$

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
0 & 0 & 0 & 0 & 0 & 1 \\
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
0 & 0 & t \\
\end{bmatrix}
\begin{bmatrix}
\ddot{x}_n \\
\ddot{y}_n \\
\ddot{\theta}_n \\
\end{bmatrix}
$


</center>

