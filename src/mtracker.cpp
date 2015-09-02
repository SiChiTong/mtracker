#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "Robot.hpp"
#include <signal.h>

/***
  This node interconnects the high-level controller
  of the MTracker robot with the low-level driver
  implemented on the robots microprocessor. It 
  subscribes to messeges of type geometry_msgs/Twist
  published under /controls topic and publish the
  odometry information obtained from the robot as
  geometry_msgs/Pose2D message under topic /pose
  as well as geometry_msgs/Twist message under topic
  velocity.

  This node works in synchronous manner, meaning that
  it has quasi-constant cycle time set to 100 Hz.

  Mateusz Przybyla
  Chair of Control and Systems Engineering
  Faculty of Computing
  Poznan University of Technology
***/


Robot* robot = new Robot();

void shutdown(int sig)
{
  ROS_INFO("MTracker shutdown");
  delete robot; // TODO: Make sure that the robot goes to off-state
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtracker");
  ros::NodeHandle n;

  ros::Publisher  pos_pub = n.advertise<geometry_msgs::Pose2D>("/pose", 10);
  ros::Publisher  vel_pub = n.advertise<geometry_msgs::Twist>("/velocity", 10);
  ros::Subscriber ctrl_sub = n.subscribe("/controls", 10, &Robot::controlsCallback, robot);

  ros::ServiceServer trig_srv = n.advertiseService("/trigger_motors", &Robot::triggerCallback, robot);

  if (!robot->com->openPort())
  {
    ROS_INFO("Could not open COM port.");
    ros::shutdown();
  }

  robot->com->stopWheels();
  robot->com->setOdometry(0.0f, 0.0f, 0.0f);

  signal(SIGINT, shutdown);

  ROS_INFO("MTracker start");

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
