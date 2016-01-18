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

ReferenceGenerator::ReferenceGenerator() : nh_(""), nh_local_("~"), time_(0.0) {
  initialize();

  ROS_INFO("Reference generator [OK]");

  ros::Rate rate(loop_rate_);
  ros::Time time_stamp = ros::Time::now();

  while (nh_.ok()) {
    ros::spinOnce();

    double dt = (ros::Time::now() - time_stamp).toSec();
    time_stamp = ros::Time::now();

    if (!paused_) {
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
    trajectory_type = 0;

  if (!nh_local_.getParam("trajectory_paused", paused_))
    paused_ = false;

  ref_pose_pub_ = nh_.advertise<geometry_msgs::Vector3>(reference_pose_topic, 10);
  ref_velocity_pub_ = nh_.advertise<geometry_msgs::Vector3>(reference_velocity_topic, 10);

  switch (trajectory_type) {
    case 0:
      trajectory_ = new Trajectory(2.0, -1.0, -M_PI_4);
      break;
    case 1:
      trajectory_ = new LinearTrajectory(0.1, M_PI_4);
      break;
    case 2:
      trajectory_ = new HarmonicTrajectory(5.0, 0.5, 0.3, 1, 1);
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
  ref_pose_ = trajectory_->calculatePose(time_);
  ref_velocity_ = trajectory_->calculateVelocity(time_);
}

void ReferenceGenerator::publish() {
  ref_pose_pub_.publish(ref_pose_);
  ref_velocity_pub_.publish(ref_velocity_);

  tf_.setOrigin(tf::Vector3(ref_pose_.x, ref_pose_.y, 0.0));
  tf_.setRotation(tf::createQuaternionFromYaw(ref_pose_.z));
  tf_br_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "world", "reference"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "reference_generator");
  ReferenceGenerator RG;
  return 0;
}
