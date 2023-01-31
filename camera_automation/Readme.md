# Command from console
- roscore
- rosrun camera_automation cam_node.py camera_serial_number exposure_time path

Example: `rosrun camera_automation cam_node.py 42120643 2000000 /home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/arv_im.png`

# Issue
```
[FATAL] [1675167467.133114]: unable to register service [/Camera- 42120643/get_loggers] with master: ERROR: parameter [service] contains illegal chars
[FATAL] [1675167467.137277]: unable to register service [/Camera- 42120643/set_logger_level] with master: ERROR: parameter [service] contains illegal chars
```
