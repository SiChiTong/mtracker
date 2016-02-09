# This script sets the ROS environmental variables for local computer, sshs into mtracker and starts the onboard script there. Then it launches the mtracker with a certain roslaunch file.
# It is advised to create an alias for the script execution, e.g. alias mtracker_start="source /home/user/catkin_ws/src/mtracker/mtracker_ui_start.sh".
# The file should be in folder /home/use/catkin_ws/src/mtracker/scripts/.

echo "Starting MTracker";

export ROS_HOSTNAME="MTrackerUI";
export ROS_IP="MTrackerUI";
export ROS_MASTER_URI="http://MTracker:11311";
export ROSLAUNCH_SSH_UNKNOWN="1";

echo "ROS_MASTER_URI: " $ROS_MASTER_URI
echo "ROS_IP: " $ROS_IP

echo "Starting roscore on MTracker. Please wait."
ssh mtracker@MTracker "source /home/mtracker/catkin_ws/src/mtracker/scripts/mtracker_start.sh" &
sleep 5;
echo "Roscore OK";

roslaunch mtracker mtracker_manual.launch
