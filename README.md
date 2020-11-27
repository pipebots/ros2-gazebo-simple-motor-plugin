# ros2-gazebo-dev-plugin

ROS2 Foxy Gazebo 11 repo to develop a new Gazebo plugin.

Setup instructions are in `docker/README.md`.

## Developing the plugin

I spent an hour or so looking for possible solutions to things that could help.

* Motors:<https://github.com/nilseuropa/gazebo_ros_motors>
This could be converted to ROS2.
* Diff-drive:
<https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_plugins/src/gazebo_ros_diff_drive.cpp>
This is known to work so could be used as a guide.  This is guide to the changes needed.
<https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Diff-drive>
* This shows how to create a ROS plugin.  It would need to be updated for ROS2.
<http://gazebosim.org/tutorials/?tut=ros_plugins#Tutorial:RosPlugins>

The ROS motors plugin looks too complicated for the simple testing.

Define new interface, something like this:

Topic: /motors

```text
uint8 motor  # Index of motor, 0 is first motor.
int16 rpm    # RPM of motor. +ve is clockwise, -ve is anti-clockwise.
```

More parameters and options can be added later.

