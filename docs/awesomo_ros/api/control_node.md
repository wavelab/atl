# Control Node

- Node Name: `control_node`
- Node Rate: 100


## Publish Topics

- `/mavros/cmd/arming`
- `/mavros/set_mode`
- `/mavros/setpoint_attitude/attitude`
- `/mavros/setpoint_attitude/att_throttle`
- `/mavros/setpoint_position/local`
- `/atl/position_controller/stats`
- `/atl/kf_estimation/stats`
- `/atl/kf_estimation/states`


## Subscribe Topics

- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/rc/in`
- `/atl/vision/pose`
