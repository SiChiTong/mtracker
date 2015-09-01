#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>

/***
  This node provides the user with an automatic controller
  for semi-autonomous motion of MTracker. It requires the
  position of the robot as a geometry_msgs/Pose2D message
  being published under topic /pos. It also needs the
  reference trajectory published as a geometry_msgs/Pose2D
  message under topic /reference. As a result it publishes
  control signals under topic /controls.

  Mateusz Przybyla
  Chair of Control and Systems Engineering
  Faculty of Computing
  Poznan University of Technology
***/


void shutdown(int sig)
{
  ROS_INFO("MTracker automatic controller shutdown");

  ros::shutdown();
}


class Controller {
public:
  float x, y, theta;
  float t;
  geometry_msgs::Twist controls;

  ros::Publisher  ctrl_pub;
  ros::Subscriber pos_sub;

  void posCallback(const geometry_msgs::Pose2D::ConstPtr& pos_msg)
  {
    this->x = pos_msg->x;
    this->y = pos_msg->y;
    this->theta = pos_msg->theta;
  }

  void computeControls()
  { 

    // HERE PUT THE CODE
  
    controls.linear.x = 1.0;
    controls.angular.z = 0.0;
  }
};


int main(int argc, char **argv)
{
  Controller c;

  ros::init(argc, argv, "mtracker_controller");
  ros::NodeHandle n;

  c.ctrl_pub = n.advertise<geometry_msgs::Twist>("/controls", 10);
  c.pos_sub = n.subscribe("/pos", 10, &Controller::posCallback, &c);

  ROS_INFO("MTracker Controller");

  signal(SIGINT, shutdown);

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();

    c.computeControls();
    c.ctrl_pub.publish(c.controls);

    rate.sleep();
  }

  return 0;
}
