# Forword

Currently I am only considering the magnetometer, accelerometer and
gyroscopes in the measurement and motion model. We will be adding gps,
barometer and other states into the ekf.

## Motion Model

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
