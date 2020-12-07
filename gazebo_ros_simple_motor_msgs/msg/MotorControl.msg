# Control modes.
int8 MODE_ABSOLUTE=0
int8 MODE_RELATIVE=1
int8 MODE_SPEED=2

# The mode to use.  One of the above mode values.
int8 mode

# The angle in degrees.
# +ve rpm is counter-clockwise when looking at the motor from the drive shaft
# end.
# When mode = MODE_ABSOLUTE, set the angle relative to the starting position.
# When mode = MODE_REVOLUTION, move the motor angle degrees relative to the
# current position.
float64 angle

# When mode = MODE_SPEED, the motor will change speed to the given RPM.
# The change in speed is controlled by the SDF file parameters update_rate and
# max_change_rpm.
# +ve rpm is counter-clockwise when looking at the motor from the drive shaft
# end.  This is one of those un-written standards.
float64 rpm
