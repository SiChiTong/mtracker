#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>

/***
 TODO

  Mateusz Przybyla
  Chair of Control and Systems Engineering
  Faculty of Computing
  Poznan University of Technology
***/



int main(int argc, char **argv)
{
  ros::init(argc, argv, "automaticgggcontroller");

  ROS_INFO("MTracker Controller");

  return 0;
}
