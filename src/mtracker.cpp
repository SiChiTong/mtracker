#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "Robot.hpp"
#include <signal.h>

Robot* robot = new Robot();

void shutdown(int sig)
{
  ROS_INFO("MTracker ROS driver shutdown");
  delete robot;
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtracker");
  ros::NodeHandle n;

  ros::Publisher  pos_pub = n.advertise<geometry_msgs::Pose2D>("/pos", 10);
  ros::Publisher  vel_pub = n.advertise<geometry_msgs::Twist>("/vel", 10);
  ros::Subscriber ctrl_sub = n.subscribe("/controls", 10, &Robot::controlsCallback, robot);

  ros::ServiceServer trig_srv = n.advertiseService("/trigger_motors", &Robot::triggerCallback, robot);

  ROS_INFO("MTracker ROS driver start");

  robot->com->openPort();
  robot->com->stopWheels();
  robot->com->setOdometry(0.0f, 0.0f, 0.0f);

  signal(SIGINT, shutdown);

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();

    if (robot->com->readFrame())
    {
      robot->pos_odom = robot->com->getPose();
      robot->vel_odom = robot->com->getVelocity();

      robot->publishPose(pos_pub);
      robot->publishVelocity(vel_pub);
    }

    if (robot->motors_on)
      robot->com->setVelocity(robot->w_l, robot->w_r);
    else
      robot->com->switchOffMotors();

    rate.sleep();
  }

  return 0;
}
