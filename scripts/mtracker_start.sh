echo "---------------------------";
echo "     Starting MTracker     ";
echo "---------------------------";
source /home/tysik/workspace/catkin_ws/src/mtracker/scripts/mtracker_ui_env.sh;
echo " ROS_MASTER_URI: " $ROS_MASTER_URI
echo " ROS_HOSTNAME: " $ROS_HOSTNAME
echo " ROS_IP: " $ROS_IP

echo "---------------------------";
echo "     Starting roscore      ";
echo "        Please wait        ";
echo "---------------------------";
ssh mtracker@MTracker "source /opt/ros/hydro/setup.bash && source /home/mtracker/catkin_ws/src/mtracker/scripts/mtracker_env.sh && roscore" &

sleep 5;
echo "---------------------------";
echo "      Starting Nodes       ";
echo "---------------------------";
roslaunch mtracker mtracker_manual_scanner.launch

