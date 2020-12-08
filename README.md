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

The ROS motor plugin looks too complicated for the simple testing.

Define new interface, something like this:

Topic: /cmd_motor

```text
# +ve rpm is counter-clockwise when looking at the motor from the drive shaft end.
# This is one of those un-written standards.
float64 rpm
```

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

Plugin to Gazebo: This is done by calling SetVelocity().

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

Abstracted out the SimpleMotor class from the GazeboRosSimplePrivate
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
[INFO] [1607100484.053219039] [test.simple_motor]: Received: motor 1, rpm -1000.000000
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
Very amusing but wrong!  Fixed.

Tested the plugin and it all works.  Tidied up debug/logging output.

### Adding angle control

After getting the basics working, I decided that it would be useful to do some
sort of slow motion control.  The use case is where the motor is being used to
directly control a non-trivial mechanism such as a cam that drives a model
over the ground.

After some thought, I settled on adding an option to set the position of the
motor directly and to also move the motor through a specified number of
degrees.  After a conversation with Andy Barber, I decided to use radians
instead of degrees.

I modified the message first, fairly easy but I needed to add a `mode` value
to tell the plugin how to use the `angle_radians` and `rpm` variables.  I then
added code to handle the extra messages that called two new functions in
`SimpleMotor` and then started implementation of the extra logic.

Relative mode works fine.

```text
[INFO] [1607457245.476815756] [test.simple_motor]: Received: mode 1, rpm 0.000000, angle_radians 2.000000
MoveRelative: delta 2.000000 radians
MoveRelative: position_radians 243.340101, target_angle_radians_ 245.340101, delta_radians_ 2.000000
UpdatePosition: next_position_radians 243.968102, target_angle_radians_ 245.340101, delta_radians_ 1.372000
UpdatePosition: next_position_radians 244.596100, target_angle_radians_ 245.340101, delta_radians_ 0.744000
UpdatePosition: next_position_radians 245.224101, target_angle_radians_ 245.340101, delta_radians_ 0.116000
UpdatePosition: next_position_radians 245.340101, target_angle_radians_ 245.340101, delta_radians_ 0.000000
```

The motor oscillates when absolute mode is used.

```text
[INFO] [1607456837.122404382] [test.simple_motor]: Received: mode 0, rpm 0.000000, angle_radians 2.000000
MoveAbsolute: new position 2.000000 radians
MoveAbsolute: position_radians 243.341506, target_angle_radians_ 2.000000, delta_radians_ 0.561128
UpdatePosition: next_position_radians 243.969506, target_angle_radians_ 2.000000, delta_radians_ -0.066872
UpdatePosition: next_position_radians 243.341504, target_angle_radians_ 2.000000, delta_radians_ 0.561128
UpdatePosition: next_position_radians 243.969505, target_angle_radians_ 2.000000, delta_radians_ -0.066872
UpdatePosition: next_position_radians 243.341506, target_angle_radians_ 2.000000, delta_radians_ 0.561128
UpdatePosition: next_position_radians 243.969505, target_angle_radians_ 2.000000, delta_radians_ -0.066872
UpdatePosition: next_position_radians 243.341503, target_angle_radians_ 2.000000, delta_radians_ 0.561128
```

The problem was that the target angle was way smaller than the actual position.
A quick modification solved that problem and both modes work correctly.

### Test/example program

The motor now needs a test program to verify the operation of the motor and to
demonstrate how it should be used.  Realised that the program could be very
simple so it was pretty straight forward to write.

Found this problem during testing.

```text
[INFO] [1607465016.592714465] [test.simple_motor]: Received: mode 0, rpm 0.000000, angle_radians -3.141593
MoveAbsolute: new position -3.141593 radians
MoveAbsolute: position_radians -0.000001, target_angle_radians_ -3.141593, delta_radians_ -3.141592
UpdatePosition: next_position_radians -0.314001, target_angle_radians_ -3.141593, delta_radians_ -2.827592
UpdatePosition: next_position_radians -0.628003, target_angle_radians_ -3.141593, delta_radians_ -2.513592
UpdatePosition: next_position_radians -0.942002, target_angle_radians_ -3.141593, delta_radians_ -2.199592
UpdatePosition: next_position_radians -1.256002, target_angle_radians_ -3.141593, delta_radians_ -1.885592
UpdatePosition: next_position_radians -1.570000, target_angle_radians_ -3.141593, delta_radians_ -1.571592
UpdatePosition: next_position_radians -1.884003, target_angle_radians_ -3.141593, delta_radians_ -1.257592
UpdatePosition: next_position_radians -2.198004, target_angle_radians_ -3.141593, delta_radians_ -0.943592
UpdatePosition: next_position_radians -2.512005, target_angle_radians_ -3.141593, delta_radians_ -0.629592
UpdatePosition: next_position_radians -2.826008, target_angle_radians_ -3.141593, delta_radians_ -0.315592
UpdatePosition: next_position_radians -3.140011, target_angle_radians_ -3.141593, delta_radians_ -0.001592
UpdatePosition: next_position_radians -3.141593, target_angle_radians_ -3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians -3.141593, target_angle_radians_ -3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 0.313998, target_angle_radians_ -3.141593, delta_radians_ -0.314000
UpdatePosition: next_position_radians 0.000000, target_angle_radians_ -3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 0.313997, target_angle_radians_ -3.141593, delta_radians_ -0.314000
UpdatePosition: next_position_radians -0.000004, target_angle_radians_ -3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 0.313997, target_angle_radians_ -3.141593, delta_radians_ -0.314000
UpdatePosition: next_position_radians -0.000001, target_angle_radians_ -3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 0.313998, target_angle_radians_ -3.141593, delta_radians_ -0.314000
UpdatePosition: next_position_radians -0.000003, target_angle_radians_ -3.141593, delta_radians_ 0.000000
[INFO] [1607465018.590537689] [test.simple_motor]: Received: mode 0, rpm 0.000000, angle_radians 3.141593
MoveAbsolute: new position 3.141593 radians
MoveAbsolute: position_radians -0.000003, target_angle_radians_ 3.141593, delta_radians_ 3.141596
UpdatePosition: next_position_radians 0.313999, target_angle_radians_ 3.141593, delta_radians_ 2.827596
UpdatePosition: next_position_radians 0.627999, target_angle_radians_ 3.141593, delta_radians_ 2.513596
UpdatePosition: next_position_radians 0.941997, target_angle_radians_ 3.141593, delta_radians_ 2.199596
UpdatePosition: next_position_radians 1.255992, target_angle_radians_ 3.141593, delta_radians_ 1.885596
UpdatePosition: next_position_radians 1.569990, target_angle_radians_ 3.141593, delta_radians_ 1.571596
UpdatePosition: next_position_radians 1.883993, target_angle_radians_ 3.141593, delta_radians_ 1.257596
UpdatePosition: next_position_radians 2.197994, target_angle_radians_ 3.141593, delta_radians_ 0.943596
UpdatePosition: next_position_radians 2.511995, target_angle_radians_ 3.141593, delta_radians_ 0.629596
UpdatePosition: next_position_radians 2.825999, target_angle_radians_ 3.141593, delta_radians_ 0.315596
UpdatePosition: next_position_radians 3.140002, target_angle_radians_ 3.141593, delta_radians_ 0.001596
UpdatePosition: next_position_radians 3.141593, target_angle_radians_ 3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 3.141593, target_angle_radians_ 3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 0.314002, target_angle_radians_ 3.141593, delta_radians_ -0.314000
UpdatePosition: next_position_radians 0.000001, target_angle_radians_ 3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 0.314003, target_angle_radians_ 3.141593, delta_radians_ -0.314000
UpdatePosition: next_position_radians 0.000004, target_angle_radians_ 3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 0.314003, target_angle_radians_ 3.141593, delta_radians_ -0.314000
UpdatePosition: next_position_radians 0.000002, target_angle_radians_ 3.141593, delta_radians_ 0.000000
UpdatePosition: next_position_radians 0.314004, target_angle_radians_ 3.141593, delta_radians_ -0.314000
[INFO] [1607465020.590603391] [test.simple_motor]: Received: mode 0, rpm 0.000000, angle_radians -3.000000
MoveAbsolute: new position -3.000000 radians
MoveAbsolute: position_radians 0.314005, target_angle_radians_ -3.000000, delta_radians_ -3.314005
```

This is only happens on absolute angle changes for -ve and -ve values of PI.
The motor moves to the correct position then jumps to near 0.  This was caused
by a floating point rounding error that occurred when the value returned by the
joint was slightly larger than + or - PI.  Fixed by adding a small value to
the fmod().

## Conclusion

This project has given me good insight into how to develop a very basic Gazebo
plugin.  Multiple instances of the motor can be used by using different
namespace values in the `model.sdf` file.

I hope you find it useful!
