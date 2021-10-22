# Relative Cartesian position controllers
Execute relative movements in the Cartesian space
It is a action server (action name `[HW_NAMEPACE]`/`[CONTROLLER_NAMESPACE]`/relative_move) of type [relative_cartesian_controller_msgs/RelativeMove](relative_cartesian_controller_msgs/action/RelativeMove.action)

Example of configuration
``` yaml
cartesian_position:
  appenders: [file, screen]
  levels: [debug, info]
  pattern_layout: "[%5p][%d{HH:mm:ss,SSS}][%M:%L][%c] %m%n"
  file_name: "cartesian_position"

  type: cnr/control/CartesianPositionController
  controlled_joints : all
  kin_update_period : 0.002

  # optional
  max_cartesian_linear_speed:         0.25  # m/s. default 0.250 m/s
  max_cartesian_linear_acceleration:  0.75  # m/s^2. default 0.750 m/s^2
  max_cartesian_angular_speed:        0.5   # rad/s. default 0.50 rad/s
  max_cartesian_angular_acceleration: 1.5   # rad/s^2. default 1.50 rad/s^2
  cartesian_angular_tolerance:        0.001 # m, goal reached when the pose error is less than cartesian_linear_tolerance. default: 0.001 m
  cartesian_angular_tolerance:        0.01  # rad, goal reached when the pose error is less than cartesian_angular_tolerance. default: 0.01 rad
  clik_gain:                          5.0  # CLIK gain. target twist = clik_gain * geometrical_error
  check_actual_configuration:         true # if true the algorithm stops when the actual position reaches the destination, if false the algorithm stops when the target reaches the destination. default true
```
