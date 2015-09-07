#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "Robot.hpp"
#include <signal.h>

/***
 * This node interconnects the high-level controller
 * of the MTracker robot with the low-level driver
 * implemented on the robots microprocessor. It
 * subscribes to messeges of type geometry_msgs/Twist
 * published under /controls topic and publish the
 * odometry information obtained from the robot as
 * geometry_msgs/Pose2D message under topic /pose
 * as well as geometry_msgs/Twist message under topic
 * velocity.

 * This node works in synchronous manner, meaning that
 * it has quasi-constant cycle time set to 100 Hz.

 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/


Robot* robot = new Robot();

void shutdown(int sig)
{
  ROS_INFO("MTracker shutdown");
  delete robot;
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtracker");
  ros::NodeHandle n;

  ros::Publisher  pose_pub = n.advertise<geometry_msgs::Pose2D>("/pose", 10);
  ros::Publisher  velocity_pub = n.advertise<geometry_msgs::Twist>("/velocity", 10);
  ros::Subscriber controls_sub = n.subscribe("/controls", 10, &Robot::controlsCallback, robot);
  ros::ServiceServer trig_srv = n.advertiseService("/trigger_motors", &Robot::triggerCallback, robot);

  if (!robot->com->openPort())
  {
    ROS_INFO("Could not open COM port.");
    ros::shutdown();
  }

  robot->switchMotors(true);
  robot->setWheelsVelocities(0.0f, 0.0f);
  robot->setOdometryPose(0.0f, 0.0f, 0.0f);

  signal(SIGINT, shutdown);

  ROS_INFO("MTracker start");

  ros::Rate rate(100.0);
  while (ros::ok())
  {
    ros::spinOnce();

    if (robot->com->readFrame())
    {
      pose_pub.publish(robot->getRobotPose());
      velocity_pub.publish(robot->getRobotVelocity());

      if (true)
      {
        tf::TransformBroadcaster pose_bc;
        tf::Transform pose_tf;
        geometry_msgs::Pose2D pose = robot->getRobotPose();

        pose_tf.setOrigin(tf::Vector3(pose.x, pose.y, 0.0));
        pose_tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, pose.theta));
        pose_bc.sendTransform(tf::StampedTransform(pose_tf, ros::Time::now(), "/world", "/robot"));
      }
    }

    rate.sleep();
  }

  return 0;
}
