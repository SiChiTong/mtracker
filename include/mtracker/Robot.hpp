#ifndef ROBOT_H
#define ROBOT_H

/* !ALL UNITS ARE IN SI! */

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "mtracker/Trigger.h"
#include "Serial.hpp"
#include <memory>

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
      com->prepareFrame(MODE_MOTORS_ON, CMD_SET_WHEELS_AND_ODOM);
    else
      com->prepareFrame(MODE_MOTORS_OFF, CMD_SET_WHEELS_AND_ODOM);

    com->writeFrame();
  }

  void setWheelsVelocities(float w_l, float w_r)
  {
    com->tx_frame.w_l =  (int16_t) (w_l * 2048.0f);
    com->tx_frame.w_r = -(int16_t) (w_r * 2048.0f);

    com->prepareFrame(MODE_MOTORS_ON, CMD_SET_WHEELS_AND_ODOM);
    com->writeFrame();
  }

  void setOdometryPose(float x, float y, float theta)
  {
    com->tx_frame.x = x;
    com->tx_frame.y = y;
    com->tx_frame.theta = theta;

    com->prepareFrame(MODE_SET_ODOMETRY | MODE_MOTORS_ON, CMD_SET_WHEELS_AND_ODOM);
    com->writeFrame();
  }

  geometry_msgs::Pose2D getRobotPose()
  {
    geometry_msgs::Pose2D pose;

    pose.x = com->rx_frame.x;
    pose.y = com->rx_frame.y;
    pose.theta = com->rx_frame.theta;

    return pose;
  }

  geometry_msgs::Twist getRobotVelocity()
  {
    geometry_msgs::Twist velocity;

    float w_l = com->rx_frame.w_l / 2048.0f;
    float w_r = com->rx_frame.w_r / 2048.0f;

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
    switchMotors(req.motors_on);
    return true;
  }
};
 
#endif
