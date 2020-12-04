# ros2-gazebo-dev-plugin

ROS2 Foxy Gazebo 11 repo to develop a new Gazebo plugin.

Setup and basic test instructions are in `docker/README.md`.

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
float64 rpm  # RPM of motor.  +ve is clockwise, -ve is anti-clockwise.
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

One thing I'm finding is that the Gazebo documentation is nearly all out of
date.  ROS2 plugin tutorials don't exist!  Ignition (the replacement for
Gazebo) has almost no documentation.

After a batch of renaming to make the code match the approach used by
`diff_drive`, decided to implement subscribe to listen to the new message and
produce log output.

One thing that caused me a load of grief was getting the new MotorControl
messages to be used.  After a lot of messing around trying to get them into
the same package, I moved the messages out to a new package as I have done
before and it worked.

### How to run the new plugin?

Trying to follow this:
<http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5>

Decided to go down the route of setting up the Gazebo environment variables
the way I want to use them.  This worked.

I can now run the plugin (instructions in `docker/README.md`), I can see the
log info on the gazebo terminal and can see the ROS subscriber is set up as
well.

Now to make the plugin interact with the model.

The tutorial I was trying to follow is so out of date it should be removed
from the internet!  I ended up using the diff_drive plugin as a model and
had much more success.

After a lot of messing around, I have a spinning box on the end of a round
motor shaft.  The interesting thing is trying to work out how to convert
rpm into a velocity value.  A velocity value of 5 = 49rpm.  A search on the
internet shows that values I have measured show that velocity in radians per
second.  The formula to convert is: 1 rad/s = 9.55 r/min (rpm).

Abstracted out the SimpleMotor class from the GazeboRosSimpleMotorsPrivate
class to allow me to change motor implementations at a future date.

Connected ROS message to update function.  A few bugs found.  This is the
first:

```text
Update: 1 current 1.775516, target0.000000, next 0.000000
Update: 1 current 0.199703, target0.000000, next 0.000000
Update: 1 current 0.024860, target0.000000, next 0.000000
Update: 2 current -0.002693, target0.000000, next 1.997307
Update: 1 current 1.771486, target0.000000, next 0.000000
```

Annoying glitch when the motor is stationary.  Added a minimum speed of
0.1 rpm to stop this happening.

Next bug:

```text
[INFO] [1607100484.053219039] [test.simple_motors]: Received: motor 1, rpm -1000.000000
Update: 3 current -0.000861, target240.000000, next -2.000861
Update: 3 current -1.777698, target240.000000, next -3.777698
Update: 3 current -3.545638, target240.000000, next -5.545638
Update: 3 current -5.322279, target240.000000, next -7.322279
```

Target rpm should be -240 not +240.  Fixed in SetSpeed() once I realised that
std::signbit() returns true when negative.

Then there is this problem:

```text
Update: 3 current -1437.276801, target-100.000000, next -1439.276801
Update: 3 current -1453.661664, target-100.000000, next -1455.661664
Update: 3 current -1460.489067, target-100.000000, next -1462.489067
```

The speed of rotation caused the motor to bounce around the simulation.
Very amusing but wrong!  Also related to sign bit so fixed that.

