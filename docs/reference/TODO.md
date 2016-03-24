# Plan

Landing a quadrotor on a moving platform.


## Milestones
- Literature review
- Make proposal
- Identify hardware (camera, extra sensors...) - Complete
- Identify potential algorithmic solutions
- Implement solutions in softwaree - Half done
- Implement solutions in hardware - Done?
- Calibration of sensors - Doneish
- Computer vision ID landing zone
- Motion Planning to land
- Perform the above successfully in simulation - Dropped, fuck mavros
- Perform the above successfully in real life



## Potential Problems

- Gazebo may have poor wind simulations
- Quadrotor ground effects (GP calcuting the error term?)



## Questions

- What should we do with cameras? Mount them
- What needs to be done on the quadrotor?
- Would it be beneficial to outsource processing.
- Should we stream data?

## Proposal:
only a smaller fraction of the image is actually needed. Id the april tag location in the image and cut an area equal to 1/4 total image 
As quad gets closer, we do not need all of the information, drop it shrink the image size. Again, chop out the area around the april tag in a large box twice the size of it.



