# Control Node

- Node Name: `control_node`
- Node Rate: 100


## Publish Topics

- `/mavros/cmd/arming`
- `/mavros/set_mode`
- `/mavros/setpoint_attitude/attitude`
- `/mavros/setpoint_attitude/att_throttle`
- `/mavros/setpoint_position/local`
- `/awesomo/position_controller/stats`
- `/awesomo/kf_estimation/stats`
- `/awesomo/kf_estimation/states`


## Subscribe Topics

- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/rc/in`
- `/awesomo/vision/pose`
