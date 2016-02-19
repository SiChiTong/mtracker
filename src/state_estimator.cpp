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

#include "../include/state_estimator.h"

using namespace mtracker;

StateEstimator::StateEstimator() : nh_(""), nh_local_("~"), state_estimator_active_(true) {
  initialize();

  ROS_INFO("Automatic controller [OK]");

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    if (state_estimator_active_) {
      estimateState();
      pose_pub_.publish(pose_);
      velocity_pub_.publish(velocity_);
      publishPoseStamped();
      publishTransform();
    }

    rate.sleep();
  }
}

void StateEstimator::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  std::string scaled_controls_topic;
  if (!nh_.getParam("scaled_controls_topic", scaled_controls_topic))
    scaled_controls_topic = "scaled_controls";

  std::string odom_pose_topic;
  if (!nh_.getParam("odom_pose_topic", odom_pose_topic))
    odom_pose_topic = "odom_pose";

  std::string optitrack_pose_topic;
  if (!nh_.getParam("optitrack_pose_topic", optitrack_pose_topic))
    optitrack_pose_topic = "optitrack_pose";

  std::string velocity_topic;
  if (!nh_.getParam("velocity_topic", velocity_topic))
    velocity_topic = "velocity";

  std::string pose_topic;
  if (!nh_.getParam("pose_topic", pose_topic))
    pose_topic = "pose";

  if (!nh_.getParam("world_frame", world_frame_))
    world_frame_ = "world";

  if (!nh_local_.getParam("child_frame", child_frame_))
    child_frame_ = "robot";

  scaled_controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(scaled_controls_topic, 10, &StateEstimator::controlsCallback, this);
  odom_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(odom_pose_topic, 10, &StateEstimator::odomPoseCallback, this);
  optitrack_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(optitrack_pose_topic, 10, &StateEstimator::optitrackPoseCallback, this);

  pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(pose_topic, 10);
  pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic + "_stamped", 10);
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(velocity_topic, 10);
  trigger_srv_ = nh_.advertiseService("state_estimator_trigger_srv", &StateEstimator::trigger, this);
}

void StateEstimator::estimateState() {
  pose_ = optitrack_pose_;
  velocity_ = controls_;
}

void StateEstimator::publishTransform() {
  pose_tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
  pose_tf_.setRotation(tf::createQuaternionFromYaw(pose_.theta));
  pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), world_frame_, child_frame_));
}

void StateEstimator::publishPoseStamped() {
  geometry_msgs::PoseStamped pose;

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = child_frame_;

  pose.pose.position.x = pose_.x;
  pose.pose.position.y = pose_.y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.theta);

  pose_stamped_pub_.publish(pose);
}

void StateEstimator::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

void StateEstimator::odomPoseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  odom_pose_ = *pose_msg;
}

void StateEstimator::optitrackPoseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  optitrack_pose_ = *pose_msg;
}

bool StateEstimator::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  state_estimator_active_ = req.activate;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_estimator");
  StateEstimator se;
  return 0;
}
