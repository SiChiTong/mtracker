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

#include "../include/simulator.h"

using namespace mtracker;

Simulator::Simulator() : nh_(""), nh_local_("~"), simulator_active_(false) {
  initialize();

  ROS_INFO("MTracker simulator [OK]");

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    if (simulator_active_) {
      computeVelocity();
      computePose();
      publish();
    }

    rate.sleep();
  }
}

void Simulator::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  if (loop_rate_ > 0)
    Tp_ = 1.0 / loop_rate_;

  if (!nh_.getParam("scaled_controls_topic", scaled_controls_topic_))
    scaled_controls_topic_ = "scaled_controls";

  if (!nh_.getParam("virtual_pose_topic", virtual_pose_topic_))
    virtual_pose_topic_ = "virtual_pose";

  if (!nh_.getParam("virtual_velocity_topic", virtual_velocity_topic_))
    virtual_velocity_topic_ = "virtual_velocity";

  if (!nh_local_.getParam("time_constant", Tf_))
    Tf_ = 0.1;

  if (!nh_local_.getParam("time_delay", To_))
    To_ = 0.0;

  if (To_ > Tp_) {
    lagged_pose_.assign(static_cast<int>(To_ / Tp_), geometry_msgs::Pose2D());
    lagged_velocity_.assign(static_cast<int>(To_ / Tp_), geometry_msgs::Twist());
  }
  else  {
    lagged_pose_.assign(1, geometry_msgs::Pose2D());
    lagged_velocity_.assign(1, geometry_msgs::Twist());
  }

  if (!nh_.getParam("world_frame", world_frame_))
    world_frame_ = "world";

  if (!nh_local_.getParam("child_frame", child_frame_))
    child_frame_ = "virtual";

  if (!nh_local_.getParam("initial_x", pose_.x))
    pose_.x = 0.0;

  if (!nh_local_.getParam("initial_y", pose_.y))
    pose_.y = 0.0;

  if (!nh_local_.getParam("initial_theta", pose_.theta))
    pose_.theta = 0.0;

  trigger_srv_ = nh_.advertiseService("simulator_trigger_srv", &Simulator::trigger, this);
  params_srv_ = nh_.advertiseService("simulator_params_srv", &Simulator::updateParams, this);
}

void Simulator::computeVelocity() {
  velocity_.linear.x  += Tp_ / (Tp_ + Tf_) * (controls_.linear.x - velocity_.linear.x);
  velocity_.angular.z += Tp_ / (Tp_ + Tf_) * (controls_.angular.z - velocity_.angular.z);

  lagged_velocity_.push_back(velocity_);
}

void Simulator::computePose() {
  pose_.x += Tp_ * velocity_.linear.x * cos(pose_.theta);
  pose_.y += Tp_ * velocity_.linear.x * sin(pose_.theta);
  pose_.theta += Tp_ * velocity_.angular.z;

  lagged_pose_.push_back(pose_);
}

void Simulator::publish() {
  geometry_msgs::Twist velocity = lagged_velocity_.front();
  geometry_msgs::Pose2D pose = lagged_pose_.front();
  geometry_msgs::PoseStamped pose_s;

  velocity_pub_.publish(velocity);
  pose_pub_.publish(pose);

  pose_tf_.setOrigin(tf::Vector3(pose.x, pose.y, 0.0));
  pose_tf_.setRotation(tf::createQuaternionFromYaw(pose.theta));
  pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), world_frame_, child_frame_));

  pose_s.header.stamp = ros::Time::now();
  pose_s.header.frame_id = child_frame_;
  pose_s.pose.position.x = pose.x;
  pose_s.pose.position.y = pose.y;
  pose_s.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

  pose_stamped_pub_.publish(pose_s);
}

void Simulator::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

bool Simulator::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  simulator_active_ = req.activate;

  if (req.activate) {
    controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(scaled_controls_topic_, 5, &Simulator::controlsCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(virtual_pose_topic_, 5);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(virtual_velocity_topic_, 5);
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(virtual_pose_topic_ + "_stamped", 5);
  }
  else {
    lagged_pose_.assign(lagged_pose_.size(), pose_);
    lagged_velocity_.assign(lagged_velocity_.size(), velocity_);

    controls_sub_.shutdown();
    pose_pub_.shutdown();
    velocity_pub_.shutdown();
    pose_stamped_pub_.shutdown();
  }

  return true;
}

bool Simulator::updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res) {
  if (req.params[0] >= 0.0 && req.params[1] >= 0.0) {
    Tf_ = req.params[0];
    To_ = req.params[1];

    if (To_ > Tp_) {
      lagged_pose_.assign(static_cast<int>(To_ / Tp_), pose_);
      lagged_velocity_.assign(static_cast<int>(To_ / Tp_), velocity_);
    }
    else  {
      lagged_pose_.assign(1, pose_);
      lagged_velocity_.assign(1, velocity_);
    }

    return true;
  }
  else
    return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");
  Simulator s;
  return 0;
}
