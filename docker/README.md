# Docker

This directory provides the user with a method to quickly build and test the
Gazebo model and plugin in a docker container.  Scripts are provided to
create the docker image and to start, stop and attach to the docker container.

NOTES: This docker is only suitable for the agent and Linux host builds.

## Basic operation

The script `./build.bash` creates the docker container.  Do this
just once!  This process took about 10 minutes on my PC, so get on with
something else while the image is built.

To start the container, use `./start.bash`.  This script starts the
container and leaves it running until `./stop.bash` is called. The script
`./start.bash` also mounts an external directory on to the container
directory `/home/build/ws`.  The external directory can be specified when
calling `./start.bash`, e.g.

```bash
cd ~/my-workspace
~/git/micro-ros-learning/agent/docker/start.bash `pwd`
```

If no path is given on the command line, the `start.bash` script will create
a workspace directory using the pattern `$HOME/<docker container name>_ws`.

If you need to change the mounted directory on the container, you need to
destroy the current container by stopping the container using `./stop.bash`,
remove the container using `./rm_container.bash` and then start a new
container using `.start.bash`.

When the container is running, you can get a Bash user prompt attached to the
container using `./attach.bash`.  The `./attach.bash` script can
be used to open multiple terminal sessions on the docker container by calling
the script as many times as needed.

## Setting up the workspace for the first time

In a shell that you started using `attach.bash`, run `gazebo --verbose` to
make sure that you have everything setup correctly.  This also creates the
`~/.gazebo` directory.  Shutdown Gazebo.  Run the commands

```bash
cd ~/ws
./setup_ws.bash
. ./setup_ws.bash
```

You should now be able to start Gazebo using `gazebo --verbose` and to add
the `test motor` model in to the empty world from the insert menu.

After adding an instance of the model, the plugin should start and produce
output similar to this:

```text
[INFO] [1606819477.922987992] [gazebo_ros_node]: ROS was initialized without arguments.
[INFO] [1606819477.954836133] [simple_motor]: Subscribed to [/cmd_motor]
[INFO] [1606819477.955216037] [simple_motor]: Attached to Gazebo
```

## References

<http://wiki.ros.org/docker/Tutorials/GUI>

## Known issues

The X authority code that was previously used seems to not be needed any more
and has been removed from the code.

Only tested using docker for Ubuntu 20.04LTS with ROS2 Foxy on an
Ubuntu 18.04LTS host.

Batch files need to be written for Windows PCs.
This looks useful <https://dev.to/darksmile92/run-gui-app-in-linux-docker-container-on-windows-host-4kde>
