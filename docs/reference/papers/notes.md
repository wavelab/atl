**Time-Optimal Control of a Hovering Quad-Rotor Helicopter (Lai et al - 2005)**

Basically a Time-Optimal Control problem that is initialized by using Genetic
Algorithm, to generate initial feasible solutions of the formulated Nonlinear
Programming problem, the author notes that GAs can be used for initialization
but not to find the time optimal values, because it very easily generates
infeasible inputs during the trajectory.



**Autonomous Landing on a Moving Platform (Friis et al - 2009)**

The goal of this project is to land a quadrotor onto a moving platform. The
main emphasis of this project is quadrotor control.The report documents the
process where by the authors derived and modelled the quadrotor dynamics. The
authors implemented both a PID and Linear Quadratic Controller (LQ), where the
latter performed better with the admission that this could be because more
effort was put into tuning LQ compared to PID. It is however unclear how much
better LQ is compared to PID considering the results are not significant enough
from first view. Overall this report provides a good coverage on the modeling
and implementation, however the experimental and evaluation sections could be
improved with better statistics in comparing PID and LQ controllers.



**Nonlinear Tracking and Landing Controller for Quadrotor Aerial Robots (Voos
et al - 2010)**

The contribution of this paper is a new nonlinear control algorithm based upon
a decomposition of the overall controller into a nested structure of velocity
and attitide control. The advantage as the author claims is that it is easy to
implement and the control algorithm has proven stability while taking the
nonlinearities of the dynamics directly into account. The experiments however
were all performed in simulation which makes me skeptical about how the control
algorithm would really perform in the real world.



**Real-time Monocular SLAM: Why Filter?**



**Precision Flight Control For a Multi-vehicle Quadrotor Helicopter testbed
(Hoffmann et al - 2011)**

Accurate dynamic models for deriving controllers for moderate to high speeds
have been lacking. This work presents theoretical models of quadrotor
aerodynamics with non-zero free stream velocities based on helicopter momentum
and balde elemnt theory, validate with static tests and flight data. Derived
controller was tested on STARMAC.

Quadrotor dynamics are often modeled as independent of free-stream velocity for
attitude and latitude control, this assumption is only reasonable at low
velocities. Even at moderate velocities the impact of the aerodynamic effects
resulting from variation in air speed is significant.

Two main effects are presented that have each been experimentally observed on
the STARMAC platform. The first effect is the variation in total thrust from a
rotor with free-stream velocity and angle of attack, and the second is the
effect known as "blade flapping" resulting from the differing flow velocities
experienced by advancing and retreating baldes of a rotor in translational
flight. Aerodynamic drag was not considered because it is both vehicle design
dependent and already well known.



**Automatic Take Off, Tracking and Landing of a Miniature UAV on a Moving
Carrier Vehicle (Wenzel et al - 2011)**

Used a Wii remote infrared (IR) camera and worked with an onboard 8 bit
microccontroller. Position and orientation relative to the IR pattern is
estimated at a frequency of approximately 50Hz.

Controller was PID, the success rate is 90%, failures are mostly due to
overshooting the tracking cone or to ground effets at departure or landing at
the small moving platform.

Impressive that it worked at all. But too limited, IR will not work outdoors.



**Quadrocoptor performance benchmarking using optimal control (Ritz et al -
2011)**

A numerical method for computing quadrocopter maneuvers between two states
is presented. Computed maneuvers satisfy Pntryagin's minimum principle
with respect to time-optimality. First, in order to obtain the structure
of time-optimal maneuvers, we apply the minimum principle to
a first-principles, two dimensional quadrotor model. Then we present
a numerical algorithm that enables the computation of maneuvers for
arbitrary initial and final states.

Pontryagin's minimum principle provides necessary conditinos, but not
sufficinet conditions for optimality. The principle is applied to
time-optimal quadrotor control. It follows that the thrust input is
bang-bang, and the rotational control is bang-singular, meaning that the
control input is always at full positive or negative saturation, except
during singular arcs.



**Autonomous Landing of a VTOL UAV on a Moving Platform Using Image-based
Visual Servoing (Lee et al - 2012)**

The authors of this paper used a Image-based Visual Servoing (IBVS) to track
the moving platform in 2D and generate a velocity reference command used as the
input to an adaptive sliding mode controller. The paper argues that their
method is computationall cheaper since other methods reconstruct a full 3D
representation. The system implemented has three modes i) Patrol mode ii)
IBVS-Guided Tracking Mode iii) IBVS-Guided Landing Mode (I think its pretty
self explanatory what they do, so I won't discuss them). The experiments show
promise however no comparisons against other methods where performed, thus it
is difficult to evaluate the relative performance of this system.



**Landing a VTOL Unmanned Aerial Vehicle on a Moving Platform Using Optical
Flow (Herisse et al - 2012)**

This paper introduces the use of optical flow to extract necessary realative
information, and a nonlinear controller to land of a moving platform, the
paper's focus is on the stabilization while tracking the moving platform and
the autonomous landing.

Previous work is based on obtaining a prediction of the motion of the moving
landing pad to provide a feed-forward compensation during the landing
manoeuvre. The advantage is that if a reliable predictive model of the motion
of the landing pad can be determined, the resulting performance of the landing
manoeuvre will be high. The disadvantage is obvious, it is difficult to obtain
reliable predictive model of the motion of the landing pad either because the
motion of the landing pad is difficult to obtain (e.g. stochastic nature, lack
of data, etc). In this case the aerial vehicle must rely on feedback control
strategies.

In this paper they draw inspiration from insects where they believe using
Visual Flow as a feedback for aerial vehicles in the control of motion in
dynamic environments can provide the necessary relative velocity and proximity
information needed to avoid obstacles, terrain following, land safely and more.
Their experiments mimicks the problem of landing a UAV on a deck of a moving
ship, so in one of their experiments they manually oscillated a landing
platform up and down while the quadrotor slowly converges to the landing
platform and lands [see video](https://www.youtube.com/watch?v=hl18Fykax8M).
While impressive the convergence rate was very slow as seen in the video.


Outdoor Autonomous Landing on a Moving Platform for Quadrotors using an
Omnidirectional Camera (Kim et al - 2014)

This paper explores the use of a smartphone (iPhone) to serve as an on-board
camera and processing unit, where measurements from the omnidirectional camera
are combined with a proper dyanmic model in order to estimate position and
velocity of the moving platform.

They colored the moving platform red and used basic color detection techniques
to identify the landing target (red platform), once the onboard computer
identifies the red platform the quadrotor converges and lands on the platform.
Overall it is difficult to see how this paper has done anything new other than
use a smart phone for its camera, the authors did not compare their method with
others.


**Autonomous Quadrotor Flight Using Onboard RGB-D Visual Odometry (Valenti et al
- 2014)**

Visual odometry algorithm used uses an uncertainty analysis of the depth
information to align newly observed features against a global sparse model of
previously detected 3D features. The visual odometry provides updates at
roughly 30Hz that is fused with 1kHz IMU sensor data via a Kalman Filter. The
high-rate pose estimation is used as feedback for the controller, enabling
autonomous flight. The authors developed a 4DOF path planner and implemented a
real time 3D SLAM where all the system runs on-board.

RGB-D visual odometry has not been widely used for quadrotor control, it uses a
frame-to-model registration approach to compute the transformation between two
consecutive camera pose.

First detect features in the captured scene by using Shi-Tomasi algorithm and
their 3D coordinates in the camera frame, then we align these features against
a global model of 3D features (previously detected).

Data association and filtering was performed using Gaussian Mixture Model. The
3D normal distribution model for each feature detected in the incoming RGB-D
image was extracted.

- AscTec Pelican
- Intel Core2Duo 1.86Ghz
- 4GB RAM
- FCU with 2 ARM7 microcontrollers

- Alpha Beta Filter
- Kalman Filter
- SURF features
- RANSAC on detected features

The results section was very bare, no comparison was given therefore it is
difficult to gauge the robustness of the implementation.



**Keyframe-Based Visual-Inertial SLAM Using NonLinear Optimization
(Leutenegger et al - 2015)**

Most fusion strategies use filtering, the visual robotics community has
recently turned to non-linear optimization approaches for visal SLAM, which
comes with significant advantages in quality of performance and computational
complexity. This paper introduces a method to tightly integrate visual
measurements with IMU in SLAM. An IMU error term is integrated with the
landmark reprojection error in fully probabilistic manner, resulting to a joint
non-linear cost function to be optimized. Employing the powerful concept of
"keyframes" we partially marginalize old states to maintain a bounded-sized
optimization window, ensuring real-time operation.

Strasdat et al (Real-time Monocular SLAM: Why filter? 2010), showed that in
**optimization-based** approaches provide better accuracy for the same
computational work when compared to filtering approaches for pure visual SLAM.
Maintaining a relatively sparse graph of keyframes and their associated
landmarks subject to nonlinear optimization, has become very popular.

Loosely-coupled systems limit the complexity, but disregard correlations
amongst internal states of different sensors.

Tightly-coupled approaches jointly estimate all sensor states.

This paper advocates a tightly-coupled fusion for maximal exploitation of
sensing cues and nonlinear estimation whereever possible rather than filtering
in order to reduce suboptimality due to linearization.

Contributions:

1. Employed a keyframe paradigm for drift-free estimation also when slow or no
motion at all is present.

2. Provide a fully probabilistic derivation of IMU error terms, including the
respective information matrix, relating successive image frames without
explicitly introducing states at IMU-rate.

3. At the system, hardware and algorithms for realtime SLAM, including robust
keypoint matching and outlier rejection using inertial cues was developed.

Results: They did not have a room to discuss the results but in all cases the
tightly-coupled approach yielded the least errro in translation and
orientation.

Perhaps it is an overkill for the landing project for now, the improvement is
relatively minor. The complexity in implementing this solution could take
months.


