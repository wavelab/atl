# Axis Convention

The MPU-9250 the axis convenvention used by the MPU can be converted to
the airplane convention (NED) by mapping the outputs in the following

## Mappings
\begin{equation}
    \begin{matrix}
        x = y_{acc} \\ 
        y = x_{acc} \\ 
        z = -z_{acc} \\ 
        \dot{x} = y_{gyro} \\ 
        \dot{y} = x_{gyro} \\ 
        \dot{z} = -z_{gyro}
    \end{matrix}
\end{equation}


