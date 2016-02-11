﻿/*
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

MTracker::MTracker() : nh_(""), nh_local_("~"), ROBOT_BASE(0.145), WHEEL_RADIUS(0.025) {
  initialize();

  com_ = new Serial("/dev/ttyUSB0");
  com_->open(921600);

  if (com_->isOpen()) {
    com_->setMode(MODE_SET_ODOMETRY | MODE_MOTORS_ON);
    com_->setPose(0.0, 0.0, 0.0);
    com_->setVelocities(0.0, 0.0);
    com_->writeFrame();

    ROS_INFO("MTracker [OK]");
  }
  else {
    ROS_INFO("Could not open COM port.");
    ros::shutdown();
  }

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    transferData();

    pose_pub_.publish(pose_);
    velocity_pub_.publish(velocity_);

    publishTransform();
    publishPoseStamped();

    rate.sleep();
  }
}

MTracker::~MTracker() {
  com_->setVelocities(0.0, 0.0);
  com_->setMode(MODE_MOTORS_OFF);
  com_->writeFrame();

  delete com_;
}

void MTracker::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  std::string scaled_controls_topic;
  if (!nh_.getParam("scaled_controls_topic", scaled_controls_topic))
    scaled_controls_topic = "scaled_controls";

  std::string odom_pose_topic;
  if (!nh_.getParam("odom_pose_topic", odom_pose_topic))
    odom_pose_topic = "odom_pose";

  std::string odom_velocity_topic;
  if (!nh_.getParam("odom_velocity_topic", odom_velocity_topic))
    odom_velocity_topic = "odom_velocity";

  if (!nh_local_.getParam("parent_frame", parent_frame_))
    parent_frame_ = "world";

  if (!nh_local_.getParam("child_frame", child_frame_))
    child_frame_ = "odometry";

  controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(scaled_controls_topic, 10, &MTracker::controlsCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(odom_pose_topic, 10);
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(odom_velocity_topic, 10);
  pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(odom_pose_topic + "_stamped", 10);
}

void MTracker::transferData() {
  double w_l = (controls_.linear.x - ROBOT_BASE * controls_.angular.z / 2.0) / WHEEL_RADIUS;
  double w_r = (controls_.linear.x + ROBOT_BASE * controls_.angular.z / 2.0) / WHEEL_RADIUS;

  com_->setVelocities(w_l, w_r);
  com_->writeFrame();
  com_->readFrame();

  pose_ = com_->getPose();

  w_l = com_->getVelocities().angular.x;
  w_r = com_->getVelocities().angular.y;

  velocity_.linear.x  = (w_r + w_l) * WHEEL_RADIUS / 2.0;
  velocity_.angular.z = (w_r - w_l) * WHEEL_RADIUS / ROBOT_BASE;
}

void MTracker::publishTransform() {
  pose_tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
  pose_tf_.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, pose_.theta));
  pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), parent_frame_, child_frame_));
}

void MTracker::publishPoseStamped() {
  geometry_msgs::PoseStamped pose;

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = child_frame_;

  pose.pose.position.x = pose_.x;
  pose.pose.position.y = pose_.y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.theta);

  pose_stamped_pub_.publish(pose);
}

void MTracker::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mtracker");
  MTracker mt;
  return 0;
}
