#!/bin/sh

export ROS_IP=MTrackerUI
export ROS_HOSTNAME=MTrackerUI
export ROS_MASTER_URI=http://MTracker:11311

. /opt/ros/hydro/setup.sh
. /home/$(env USER)/catkin_ws/devel/setup.sh
exec "$@"
