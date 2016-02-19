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

      velocity_pub_.publish(velocity_);
      pose_pub_.publish(pose_);

      publishTransform();
      publishPoseStamped();
    }

    rate.sleep();
  }
}

void Simulator::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  if (loop_rate_ != 0)
    Tp_ = 1.0 / loop_rate_;

  std::string scaled_controls_topic;
  if (!nh_.getParam("scaled_controls_topic", scaled_controls_topic))
    scaled_controls_topic = "scaled_controls";

  std::string virtual_pose_topic;
  if (!nh_.getParam("virtual_pose_topic", virtual_pose_topic))
    virtual_pose_topic = "virtual_pose";

  std::string virtual_velocity_topic;
  if (!nh_.getParam("virtual_velocity_topic", virtual_velocity_topic))
    virtual_velocity_topic = "virtual_velocity";

  if (!nh_local_.getParam("time_constant", Tf_))
    Tf_ = 0.1;

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

  controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(scaled_controls_topic, 10, &Simulator::controlsCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(virtual_pose_topic, 10);
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(virtual_velocity_topic, 10);
  pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(virtual_pose_topic + "_stamped", 10);
  trigger_srv_ = nh_.advertiseService("simulator_trigger_srv", &Simulator::trigger, this);
}

void Simulator::computeVelocity() {
  velocity_.linear.x  += Tp_ / (Tp_ + Tf_) * (controls_.linear.x - velocity_.linear.x);
  velocity_.angular.z += Tp_ / (Tp_ + Tf_) * (controls_.angular.z - velocity_.angular.z);
}

void Simulator::computePose() {
  pose_.x += Tp_ * velocity_.linear.x * cos(pose_.theta);
  pose_.y += Tp_ * velocity_.linear.x * sin(pose_.theta);
  pose_.theta += Tp_ * velocity_.angular.z;
}

void Simulator::publishTransform() {
  pose_tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
  pose_tf_.setRotation(tf::createQuaternionFromYaw(pose_.theta));
  pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), world_frame_, child_frame_));
}

void Simulator::publishPoseStamped() {
  geometry_msgs::PoseStamped pose;

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = child_frame_;

  pose.pose.position.x = pose_.x;
  pose.pose.position.y = pose_.y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.theta);

  pose_stamped_pub_.publish(pose);
}

void Simulator::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

bool Simulator::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  simulator_active_ = req.activate;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");
  Simulator S;
  return 0;
}
