#!/bin/bash
# Set up the workspace to build the plugin.
# This is designed to be run from inside the docker.
set -e

cd ${HOME}/ws
mkdir -p src
cd src

# Link the plugin code into the workspace.
ln -sf ~/code/gazebo_ros_simple_motor_msgs
ln -sf ~/code/gazebo_ros_simple_motor

# Do the first build.
cd ${HOME}/ws
colcon build

echo
echo "Setup took $SECONDS seconds."
