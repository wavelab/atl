# Interfacing with Optitrack

Pixhawk allows you to feed it with mocap data to provide the pixhawk with
good estimation of where it is in the inertia frame. Below we will detail
how we setup Optitrack to work with Pixhawk, other systems such as Vicon
are not covered here.

## ROS Architecture

As of 13th March 2016 we are unable to stream mocap data via wireless,
additionally the MAVROS node needs to run on the Odroid. Since both nodes
cannot run on a single computer, the alternative is to create a ROS
Master/Slave setup.
