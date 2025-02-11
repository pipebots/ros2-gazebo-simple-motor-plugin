# ROS2 Gazebo Simple Motor Plugin

A ROS2 Gazebo plugin that drives controls a specified joint to simulate
a motor.  The simulation of the motor is simple as only the rotational
velocity or the position can be set using the ROS message.

__Important note__: Gazebo needs a PC with a good graphics card to run at a
decent frame rate.  The minimum specification is stated as "a graphics card
with 4GB of RAM or greater".  The laptop I'm using has an NVidia Quadro M2000M
with 4GB RAM and is able to simulate simple models at 60FPS although it
struggles with large scale simulations.

Docker commands used for development may be found in this
[read me file](docker/README.md).

Notes on how I developed the plugin can be found in my
[development notes](development_notes.md).

## How to use the plugin

### Pre-requisites

A PC, virtual machine or docker that has:

* Ubuntu 20.04LTS
* ROS2 Foxy
* Gazebo 11

Verify that you can run successfully run ROS2 commands and start Gazebo from
the command line.  Hint: use these first!

```bash
. /opt/ros/foxy/setup.bash
. /usr/share/gazebo/setup.sh
```

### Add the packages to your project

The easiest way to use the plugin is to add the two packages,
`gazebo_ros_simple_motor` and `gazebo_ros_simple_motor_msgs`, to your existing
project.  Clone this repo in your workspace `src` directory, then build.
The plugin packages should be built and you should be good to go.

### Testing the plugin

Start Gazebo

To test the plugin using ROS2 commands, run one or more of these commands:

```bash
cd ~/ws
. ./install/local_setup.bash
$ ros2 topic list
/clock
/test/cmd_motor
/parameter_events
/rosout
$ ros2 topic info /test/cmd_motor
Type: gazebo_ros_simple_motor_msgs/msg/MotorControl
Publisher count: 0
Subscription count: 1
# Spin the motor.
$ ros2 topic pub --once /test/cmd_motor gazebo_ros_simple_motor_msgs/msg/MotorControl '{"mode": 2, "rpm": 100}'
publisher: beginning loop
publishing #1: gazebo_ros_simple_motor_msgs.msg.MotorControl(mode=2, angle_radians=0.0, rpm=100.0)
```

## Known issues

Using the Docker scripts involves many extra undocumented steps.

The released libraries have only been tested in my docker.

### Install the plugin from binaries

__FIXME These instructions don't work for some reason.  Need to retest and fix.__

Before trying to use the plugin, you need to tell Gazebo where to find the
model and the plugin.  The model can be found here `models/test_motor` and
the libraries here `release`.

__NOTE__: These exports are examples and need modifying for your setup.

```bash
MY_MODEL_PATH=~/ws/models
MY_PLUGIN_PATH=~/ws/release/gazebo_ros_simple_motor/lib:
MY_PLUGIN_PATH+=~/ws/release/gazebo_ros_simple_motor_msgs/lib:
export GAZEBO_MODEL_PATH=${MY_MODEL_PATH}:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=${MY_PLUGIN_PATH}:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${MY_PLUGIN_PATH}:${LD_LIBRARY_PATH}
```

Start Gazebo using `gazebo --verbose` and add the `test_motor` model on the
insert menu in to the empty world.  The plugin should start and
produce output similar to this:

```text
[INFO] [1606819477.922987992] [gazebo_ros_node]: ROS was initialized without arguments.
[INFO] [1606819477.954836133] [simple_motor]: Subscribed to [/cmd_motor]
[INFO] [1606819477.955216037] [simple_motor]: Attached to Gazebo
...
```

## Acknowledgments

This work is supported by the UK's Engineering and Physical Sciences Research Council (EPSRC) Programme Grant EP/S016813/1

© 2020,2025 University of Leeds.

The author, A. Blight, has asserted his moral rights.
