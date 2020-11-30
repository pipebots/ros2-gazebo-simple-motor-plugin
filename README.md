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
float64 rpm  # RPM of motor. +ve is clockwise, -ve is anti-clockwise.
```

More parameters and options can be added later.

I decided to pick apart the `gazebo_ros_diff_drive` plugin as it is widely
used and I can see how everything fits together.  What I need to focus on is:

* ROS messages into and out of the plugin.
* How the message gets converted into something that causes a wheel to move
in Gazebo.  This will probably break down to something like:
  * Identify the interfaces between the plugin and Gazebo (probably
    URDF/SDF file).
  * Work out what each part of the interfaces do and identify the part that
  causes a wheel to rotate.
* Figure out what I need to do to make my plugin.
* How does Gazebo know where to move the wheel to?  Transforms?

This information needs to be documented as I haven't found anything like this
yet.

### Review of gazebo_ros_diff_drive

The header file shows that the class `GazeboRosDiffDrive` is inherited from
`gazebo::ModelPlugin`.  This means that Gazebo can use the plugin with no
problems with interface mis-match.  The actual implementation of the
`diff_drive` functionality is implemented in a private implementation class
`GazeboRosDiffDrivePrivate` that is a standalone class.  I think this a really
neat way to isolate implementations from interfaces.

The source file defines the methods for both classes.

#### GazeboRosDiffDrive

The constructor instantiates the private implementation class and the
destructor does nothing.

The function `Reset` resets velocity and rotation for all pairs of wheels to 0.

The `Load` function does a lot more!

1. Get the ROS node and QoS.
1. Read the number of pairs of wheels from the SDF file.
1. Read the properties from the SDF file for maximum wheel acceleration and
torque.
1. Read the joint and kinematic properties from the SDF file.  This step is
quite involved and needs a second pass to understand it.  TODO
1. Read update rate from SDF file and convert to update period.
1. Create subscriber for `cmd_vel` message.
1. Read odometry information from SDF file.
1. Create publisher for `odom` topic if needed.
1. Read wheel and odometry transform values from SDF file.
1. Create transform broadcaster as needed.
1. Load covariance values.
1. Bind `OnUpdate` function to Gazebo.

#### GazeboRosDiffDrivePrivate

`OnUpdate` is called by Gazebo once every simulation iteration and
mostly calls other functions depending on internal variables set by the
`GazeboRosDiffDrive::Load` function.  The functions are:

* `UpdateOdometryEncoder`
* `UpdateOdometryWorld`
* `PublishOdometryMsg`
* `PublishWheelsTf`
* `PublishOdometryTf`
* `UpdateWheelVelocities`

After these functions have been called, the current speed for each wheel is
obtained from the joint for each wheel and any adjustments are made.

`UpdateWheelVelocities` sets the desired speed for each wheel.

`OnCmdVel` is called when a `/cmd_vel` message is received by the subscriber.
It updates the internal values `target_x_` and `target_rot_`.

`UpdateOdometryEncoder` does some clever maths stuff that results in the
`pose.pose.position`, `pose.pose.orientation`, `twist.twist.angular.z` and
`twist.twist.linear.x/y` members of the internal variable `odom_` being
updated.

`UpdateOdometryWorld` updates the `odom_` member variable using the
`WorldPose` from Gazebo.

`PublishOdometryTf` publishes a message using the values from `odom_`.

`PublishWheelsTf` publishes a transform message about the position of each of
the wheels.

`PublishOdometryMsg` publishes most of the `odom_` member variable.

`odom_` is updated in `UpdateOdometryEncoder` and `UpdateOdometryWorld`.  It
is used by `PublishOdometryTf`, `PublishOdometryMsg`.  Declared line 206.

#### Summary

The diff drive plugin seems straightforward although there are a lot of
things going on.  It is easy enough to see how the ROS messages are processed
but the connections to Gazebo are a little more tricky.

Gazebo to plugin: The plugin gains access to the `gazebo::physics::ModelPtr`
in the `Load` function and uses the Gazebo model in `UpdateOdometryWorld`.

Plugin to Gazebo: ?  TODO

## Developing the new plugin

<http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin>

Other people seem to build their plugins as ROS packages, so I'll do the same.

Setup an empty shell class based on the approach used by diff_drive.

Added a new message to control the motor.

Added subscriber and tested.

One thing I'm finding is that the Gazebo documentation is nearly all out of
date.  ROS2 plugin tutorials don't exist!  Ignition (the replacement for
Gazebo) has almost no documentation.

