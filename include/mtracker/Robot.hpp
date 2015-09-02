#ifndef ROBOT_H
#define ROBOT_H

/* !ALL UNITS ARE IN SI! */

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
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

  geometry_msgs::Pose2D pos_odom;
  geometry_msgs::Twist  vel_odom;

  float w_l, w_r;
  bool motors_on;

  Robot() : w_l(0.0f), w_r(0.0f), motors_on(false)
  {
    std::unique_ptr<Serial> c(new Serial);
    com = std::move(c);
  }
  ~Robot() {}

  void publishPose(ros::Publisher pub)
  {
    static tf::TransformBroadcaster pose_bc;
    static tf::Transform pose_tf;

    pose_tf.setOrigin(tf::Vector3(pos_odom.x, pos_odom.y, 0.0));
    pose_tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, pos_odom.theta));
    pose_bc.sendTransform(tf::StampedTransform(pose_tf, ros::Time::now(), "/world", "/robot"));

    pub.publish(pos_odom);
  }

  void publishVelocity(ros::Publisher pub)
  {
    pub.publish(vel_odom);
  }

  void controlsCallback(const geometry_msgs::Twist msg)
  {
    w_l = (msg.linear.x - ROBOT_BASE * msg.angular.z / 2.0) / WHEEL_RADIUS;
    w_r = (msg.linear.x + ROBOT_BASE * msg.angular.z / 2.0) / WHEEL_RADIUS;
  }

  bool triggerCallback(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res)
  {
    motors_on = req.motors_on;
    return true;
  }
};
 
#endif
