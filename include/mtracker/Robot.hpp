#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "mtracker/Trigger.h"
#include "Serial.hpp"
#include <memory>

/* ALL UNITS ARE IN SI */

#define ROBOT_BASE   0.145
#define WHEEL_RADIUS 0.025

class Robot
{
public:
  std::unique_ptr<Serial> com;

  geometry_msgs::Pose2D pose;
  geometry_msgs::Twist  velocity;

  Robot()
  {
    std::unique_ptr<Serial> c(new Serial);
    com = std::move(c);
  }

  ~Robot()
  {
    setWheelsVelocities(0.0f, 0.0f);
    switchMotors(false);
  }

  void switchMotors(bool motors_on)
  {
    if (motors_on)
      com->setMode(MODE_MOTORS_ON);
    else
      com->setMode(MODE_MOTORS_OFF);

    com->writeFrame();
  }

  void setWheelsVelocities(float w_l, float w_r)
  {
    com->setVelocities(w_l, w_r);
    com->setMode(MODE_MOTORS_ON);
    com->writeFrame();
  }

  void setOdometryPose(float x, float y, float theta)
  {
    com->setPose(x, y, theta);
    com->setMode(MODE_SET_ODOMETRY);
    com->writeFrame();
  }

  geometry_msgs::Pose2D getRobotPose()
  {
    return com->getPose();
  }

  geometry_msgs::Twist getRobotVelocity()
  {
    geometry_msgs::Twist velocity;

    float w_l = com->getVelocities().angular.x;
    float w_r = com->getVelocities().angular.y;

    velocity.linear.x  = (w_r + w_l) * WHEEL_RADIUS / 2.0;
    velocity.angular.z = (w_r - w_l) * WHEEL_RADIUS / ROBOT_BASE;

    return velocity;
  }

  void controlsCallback(const geometry_msgs::Twist msg)
  {
    float w_l = (msg.linear.x - ROBOT_BASE * msg.angular.z / 2.0) / WHEEL_RADIUS;
    float w_r = (msg.linear.x + ROBOT_BASE * msg.angular.z / 2.0) / WHEEL_RADIUS;

    setWheelsVelocities(w_l, w_r);
  }

  bool triggerCallback(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res)
  {
    switchMotors(req.trigger);
    return true;
  }
};
 
#endif
