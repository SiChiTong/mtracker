/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "../include/mtracker.h"

using namespace mtracker;

MTracker::MTracker() : nh_(""), nh_local_("~"), com_(new Serial("/dev/ttyUSB0")), ROBOT_BASE(0.145f), WHEEL_RADIUS(0.025f) {
  initialize();
  com_->open(921600);

  if (com_->isOpen()) {
    switchMotors(true);
    stopWheels();
    initializeOdometry(0.0f, 0.0f, 0.0f);

    ROS_INFO("MTracker start");
  }
  else {
    ROS_INFO("Could not open COM port.");
    ros::shutdown();
  }

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    transferData();
    odom_pose_pub_.publish(odom_pose_);
    odom_velocity_pub_.publish(odom_velocity_);
    publishTransform();

    rate.sleep();
  }
}

MTracker::~MTracker() {
  stopWheels();
  switchMotors(false);
  delete com_;
}

void MTracker::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

void MTracker::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  std::string controls_topic;
  if (!nh_.getParam("controls_topic", controls_topic))
    controls_topic = "controls";

  std::string odom_pose_topic;
  if (!nh_.getParam("odom_pose_topic", odom_pose_topic))
    odom_pose_topic = "odom_pose";

  std::string odom_velocity_topic;
  if (!nh_.getParam("odom_velocity_topic", odom_velocity_topic))
    odom_velocity_topic = "odom_velocity";

  controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(controls_topic, 10, &MTracker::controlsCallback, this);
  odom_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(odom_pose_topic, 10);
  odom_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(odom_velocity_topic, 10);

  com_->setPose(x, y, theta);
  com_->setMode(MODE_SET_ODOMETRY);
  com_->writeFrame();
}

void MTracker::transferData() {
  double w_l = (controls_.linear.x - ROBOT_BASE * controls_.angular.z / 2.0) / WHEEL_RADIUS;
  double w_r = (controls_.linear.x + ROBOT_BASE * controls_.angular.z / 2.0) / WHEEL_RADIUS;

  com_->setVelocities(w_l, w_r);
  com_->setMode(MODE_MOTORS_ON);

  com_->writeFrame();
  com_->readFrame();

  odom_pose_ = com_->getPose();
  odom_velocity_ = com_->getVelocities();
}

void MTracker::publishTransform() {
  pose_tf_.setOrigin(tf::Vector3(odom_pose_.x, odom_pose_.y, 0.0));
  pose_tf_.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, odom_pose_.theta));
  pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), "world", "robot"));
}


//  void publishPath() {
//    geometry_msgs::PointStamped p;
//    p.header.frame_id = "robot";
//    p.header.stamp = ros::Time::now();
//    p.point.x = pose_.x;
//    p.point.y = pose_.y;
//    path_pub_.publish(p);
//  }


void MTracker::switchMotors(bool motors_on) {
  if (motors_on)
    com_->setMode(MODE_MOTORS_ON);
  else
    com_->setMode(MODE_MOTORS_OFF);

  com_->writeFrame();
}

//  geometry_msgs::Pose2D getRobotPose() {
//    return com_->getPose();
//  }


//  geometry_msgs::Twist getRobotVelocity() {
//    geometry_msgs::Twist velocity;

//    float w_l = com_->getVelocities().angular.x;
//    float w_r = com_->getVelocities().angular.y;

//    velocity.linear.x  = (w_r + w_l) * WHEEL_RADIUS / 2.0;
//    velocity.angular.z = (w_r - w_l) * WHEEL_RADIUS / ROBOT_BASE;

//    return velocity;
//  }


int main(int argc, char** argv) {
  ros::init(argc, argv, "mtracker");
  MTracker mt;
  return 0;
}
