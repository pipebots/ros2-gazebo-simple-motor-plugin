#!/bin/bash
# Setup turtlebot simulation.
# This is designed to be run from inside the docker.
set -e

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    ros-foxy-turtlesim \
    ros-foxy-rqt


echo
echo "The app setup took $SECONDS seconds."
echo
