#!/bin/sh

export ROS_IP=MTracker
export ROS_HOSTNAME=MTracker
export ROS_MASTER_URI=http://MTracker:11311

. /opt/ros/hydro/setup.sh
. /home/$(env USER)/catkin_ws/devel/setup.sh
exec "$@"
