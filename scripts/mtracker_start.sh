# This script sets the ROS environmental variables for uPC computer and starts roscore there.
# The file should be in folder /home/float/catkin_ws/src/mtracker/scripts/

export ROS_HOSTNAME="MTracker";
export ROS_IP="MTracker";
export ROS_MASTER_URI="http://MTracker:11311";
export ROSLAUNCH_SSH_UNKNOWN="1";

source /opt/ros/hydro/setup.bash

roscore
