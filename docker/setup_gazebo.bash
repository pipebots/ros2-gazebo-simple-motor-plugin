#!/bin/bash
# Set up Gazebo models. plugins and worlds.
# This is designed to be sourced from inside the docker.

if [ ${0##*/} == ${BASH_SOURCE[0]##*/} ]; then
    echo "WARNING"
    echo "This script is not meant to be executed directly!"
    echo "Use this script only by sourcing it."
    echo
    exit 1
fi

# I tried ``. /usr/share/gazebo/setup.sh`
# but it appends most variables so cannot be called more than once.
# So instead set everything up manually.
# The plugin
MY_MODEL_PATH=~/code/models
MY_PLUGIN_PATH=~/ws/install/gazebo_ros_simple_motors/lib:
MY_PLUGIN_PATH+=~/ws/install/gazebo_ros_simple_motors_msgs/lib:
MY_RESOURCE_PATH=
export GAZEBO_MASTER_URI=http://localhost:11345
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
export GAZEBO_MODEL_PATH=${MY_MODEL_PATH}:/usr/share/gazebo-11/models:
export GAZEBO_PLUGIN_PATH=${MY_PLUGIN_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:
export GAZEBO_RESOURCE_PATH=${MY_RESOURCE_PATH}:/usr/share/gazebo-11:
# This is needed for other dependent libraries.
export LD_LIBRARY_PATH=${MY_PLUGIN_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/foxy/opt/yaml_cpp_vendor/lib:/opt/ros/foxy/opt/rviz_ogre_vendor/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:
