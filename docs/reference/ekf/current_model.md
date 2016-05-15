# Forword

Currently I am only considering the magnetometer, accelerometer and
gyroscopes in the measurement and motion model. We will be adding gps,
barometer and other states into the ekf

In the following roll, pitch yaw are denoted by $\phi, \theta, \psi$ and
rotation rates are denoted as $\dot{\phi}, \dot{\theta}, \dot{\psi} $ 
respectively

## Motion Model (g function)

\begin{equation}
    \begin{bmatrix}
        \phi_t \\
        \theta_t \\
        \psi_t \\
        \dot{\phi}_t \\
        \dot{\theta}_t \\
        \dot{\psi}_t
    \end{bmatrix} =
    \begin{bmatrix}
        \phi_{t-1} + \dot{\phi}_{t-1} dt \\
        \theta_{t-1} + \dot{\theta}_{t-1} dt \\
        \psi_{t-1} + \dot{\psi}_{t-1}dt\\
        \dot{\phi}_{t-1}\\
        \dot{\theta}_{t-1}\\
        \dot{\psi}_{t-1}
    \end{bmatrix} +
    \begin{bmatrix}
        \sigma_{\phi} \\
        \sigma_{\theta} \\
        \sigma_{\psi} \\
        \sigma_{\dot{\phi}} \\
        \sigma_{\dot{\theta}} \\
        \sigma_{\dot{\psi}}
    \end{bmatrix}
\end{equation}

## Motion Model Jacobian (G function)
This is just an 6 by 6 identity matrix as the motion model (g) is linear. 

## Measurement Model (h function)
\begin{equation}
    \begin{bmatrix}
        \phi_{mag} \\
        \theta_{mag}\\ 
        \psi_{mag}\\ 
        \phi_{acc} \\
        \theta_{acc}\\ 
        \psi_{acc}\\ 
        \dot{\phi}_{gyro} \\
        \dot{\theta}_{gyro}\\ 
        \dot{\psi}_{gyro}\\ 
    \end{bmatrix} =
    \begin{bmatrix}
        \phi_{mag} \\
        \theta_{mag}\\ 
        \psi_{mag}\\ 
        \tan^{-1} \left(x / \sqrt{y_{acc}^2 + z_{acc}^2 } \right )\\
        \tan^{-1} \left(y / \sqrt{x_{acc}^2 + z_{acc}^2 } \right )\\ 
        \tan^{-1} \left(z / \sqrt{x_{acc}^2 + y_{acc}^2 } \right )\\ 
        \dot{\phi}_{gyro} \\
        \dot{\theta}_{gyro}\\ 
        \dot{\psi}_{gyro}\\ 
    \end{bmatrix} +
    \begin{bmatrix}
        \sigma_{\phi_{mag}} \\
        \sigma_{\theta_{mag}}\\ 
        \sigma_{\psi_{mag}}\\ 
        \sigma_{\phi_{acc}} \\
        \sigma_{\theta_{acc}}\\ 
        \sigma_{\psi_{acc}}\\ 
        \sigma_{\dot{\phi}_{gyro}} \\
        \sigma_{\dot{\theta}_{gyro}}\\ 
        \sigma_{\dot{\psi}_{gyro}}\\ 
    \end{bmatrix}
\end{equation}

## Measurement Model Jacobian (H Matrix)

\begin{equation}H =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\ 
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
\frac{a}{L} & \frac{-xy}{a \cdot L} & \frac{-xz}{a \cdot L} & 0 & 0 & 0 \\
\frac{-xy}{b \cdot L} & \frac{b}{L} & \frac{-yz}{b \cdot L} & 0 & 0 & 0 \\
\frac{-xz}{c \cdot L} & \frac{-yz}{c \cdot L} & \frac{c}{L} & 0 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
\end{equation}

where:
$ a = \sqrt{y^2 + z^2} \\
  b = \sqrt{y^2 + z^2} \\
  c = \sqrt{y^2 + z^2} \\
  L = \left( x^2 + y^2 + z^2 \right ) $

and $x, y$ and $z$ are the acceleration measurements in the x, y and z directions. 



