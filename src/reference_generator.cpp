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

#include "../include/reference_generator.h"

using namespace mtracker;

ReferenceGenerator::ReferenceGenerator() : nh_(""), nh_local_("~"), reference_generator_active_(true), time_(0.0) {
  initialize();

  ROS_INFO("Reference generator [OK]");

  ros::Rate rate(loop_rate_);
  ros::Time time_stamp = ros::Time::now();

  while (nh_.ok()) {
    ros::spinOnce();

    if (reference_generator_active_) {
      double dt = (ros::Time::now() - time_stamp).toSec();
      time_stamp = ros::Time::now();

      if (!paused_)
        update(dt);

      publish();
    }
    rate.sleep();
  }
}

ReferenceGenerator::~ReferenceGenerator() {
  delete trajectory_;
}

void ReferenceGenerator::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  std::string reference_pose_topic;
  if (!nh_.getParam("reference_pose_topic", reference_pose_topic))
    reference_pose_topic = "reference_pose";

  std::string reference_velocity_topic;
  if (!nh_.getParam("reference_velocity_topic", reference_velocity_topic))
    reference_velocity_topic = "reference_velocity";

  int trajectory_type;
  if (!nh_local_.getParam("trajectory_type", trajectory_type))
    trajectory_type = 2;

  if (!nh_local_.getParam("trajectory_paused", paused_))
    paused_ = false;

  if (!nh_local_.getParam("parent_frame", parent_frame_))
    parent_frame_ = "world";

  if (!nh_local_.getParam("child_frame", child_frame_))
    child_frame_ = "reference";

  pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(reference_pose_topic, 10);
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(reference_velocity_topic, 10);
  pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(reference_pose_topic + "_stamped", 10);
  play_pause_srv_ = nh_.advertiseService("reference_play_pause_srv", &ReferenceGenerator::playPause, this);
  trigger_srv_ = nh_.advertiseService("reference_generator_trigger_srv", &ReferenceGenerator::trigger, this);

  switch (trajectory_type) {
    case 0:
      trajectory_ = new Trajectory(2.0, -1.0, -M_PI_4);
      break;
    case 1:
      trajectory_ = new LinearTrajectory(0.1, M_PI_4);
      break;
    case 2:
      trajectory_ = new HarmonicTrajectory(5.0, 0.5, 0.3, 2, 1);
      break;
    case 3:
      trajectory_ = new LemniscateTrajectory();
      break;
    default:
      trajectory_ = new Trajectory();
      break;
  }

  update(0.0);
}

void ReferenceGenerator::start() {
  paused_ = false;
}

void ReferenceGenerator::stop() {
  paused_ = true;
  time_ = 0.0;
  update(0.0);
}

void ReferenceGenerator::pause() {
  paused_ = true;
}

void ReferenceGenerator::update(double dt) {
  time_ += dt;
  pose_ = trajectory_->calculatePose(time_);
  velocity_ = trajectory_->calculateVelocity(time_);
}

void ReferenceGenerator::publish() {
  pose_pub_.publish(pose_);
  velocity_pub_.publish(velocity_);

  tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
  tf_.setRotation(tf::createQuaternionFromYaw(pose_.theta));
  tf_br_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), parent_frame_, child_frame_));

  geometry_msgs::PoseStamped pose;

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = child_frame_;

  pose.pose.position.x = pose_.x;
  pose.pose.position.y = pose_.y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.theta);

  pose_stamped_pub_.publish(pose);
}

bool ReferenceGenerator::playPause(mtracker::PlayPause::Request &req, mtracker::PlayPause::Response &res) {
  if (req.play)
    start();
  else if (req.pause)
    pause();
  else
    stop();

  return true;
}

bool ReferenceGenerator::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  reference_generator_active_ = req.activate;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "reference_generator");
  ReferenceGenerator RG;
  return 0;
}
