# Coordinate Frame Analysis

![Coordinate Frame](images/coordinate_frame.png)

\begin{align}
  R_{321} &= \begin{bmatrix}
    C(\theta) C(\psi) &
    sin(\phi) sin(\theta) C(\psi) - C(\phi) sin(\psi) &
    C(\phi) sin(\theta) C(\psi) + sin(\phi) sin(\psi)  \\
    C(\theta) sin(\psi) &
    sin(\phi) sin(\theta) sin(\psi) + C(\phi) C(\psi) &
    C(\phi) sin(\theta) sin(\psi) - sin(\phi) C(\psi) \\
    -sin(\theta) &
    sin(\phi) C(\theta) &
    C(\phi) C(\theta)
  \end{bmatrix} \\
  R_{C}^{N} &= \begin{bmatrix}
    0 & 0 & 1 \\
    -1 & 0 & 0 \\
    0 & -1 & 0
  \end{bmatrix} \\
  R_{N}^{G} &= R_{321} \\
  R_{G}^{P} &= \begin{bmatrix}
    0 & 0 & 1 \\
    0 & 1 & 0 \\
    -1 & 0 & 0
  \end{bmatrix} \\
\end{align}

Once the landing target is identified and detected using the gimballed camera
and an AprilTag detector onboard the quadrotor, the pose extracted requires a
series of transformations to convert the AprilTag measurements into a suitable
coordinate frame for tracking and landing.

First, measurements of the landing target $L$ captured in the camera body-fixed
frame $C$ are first transformed from EDN to NWU ($T_{C}^{N}$) to match the
gimbal frame coordinate system, then it is transformed from NWU to gimbal frame
$G$ ($T_{N}^{G}$), values for this transform originate from gimbal IMU attitude
readings and offsets to the quadrotor body center. A final transformation
$T_{G}^{P}$ brings the landing target pose measurement from the gimbal frame
$G$ to the body planar frame $P$, this frame allows our quadrotor flight and
gimbal controllers to uniquely track relative position errors in the inertial
aligned horizon plane, it differs from a conventional body frame $B$ that it
ignores both roll and pitch (but not yaw)

\begin{equation}
  \textbf{x}_{P} =
    T_{G}^{P}
    T_{N}^{G}
    T_{C}^{N}
    \textbf{x}_{C}
\end{equation}

In this work it is assumed that the gimbal and camera axis are precisely
aligned and that the roll and pitch motors are also directly aligned with the
camera image sensors $x$ and $y$ axes. Therefore the translation between the
camera and the gimbal may be neglected to result in the following equation:

\begin{equation}
  \textbf{x}_{P} =
    R_{G}^{P}
    R_{N}^{G}
    R_{C}^{N}
    \textbf{x}_{C}
\end{equation}

\begin{align}
  \textbf{x}_{P} &= f(\lambda) \\
  J &= \begin{bmatrix}
    \dfrac{\partial{f_{1}}}{\partial{\lambda}} \\
    \dfrac{\partial{f_{2}}}{\partial{\lambda}} \\
    \dfrac{\partial{f_{3}}}{\partial{\lambda}}
  \end{bmatrix}
\end{align}
