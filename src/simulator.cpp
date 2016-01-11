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

Simulator::Simulator() : nh_(""), nh_local_("~") {
  initialize();

  ROS_INFO("MTracker simulator start");

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    computeVelocity();
    computePose();
    velocity_pub_.publish(velocity_);
    pose_pub_.publish(pose_);
    publishTransform();

    rate.sleep();
  }
}

void Simulator::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

void Simulator::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  if (loop_rate_ != 0)
    Tp_ = 1.0 / loop_rate_;

  std::string controls_topic;
  if (!nh_.getParam("controls_topic", controls_topic))
    controls_topic = "controls";

  std::string virtual_pose_topic;
  if (!nh_.getParam("virtual_pose_topic", virtual_pose_topic))
    virtual_pose_topic = "virtual_pose";

  std::string virtual_velocity_topic;
  if (!nh_.getParam("virtual_velocity_topic", virtual_velocity_topic))
    virtual_velocity_topic = "virtual_velocity";

  if (!nh_local_.getParam("time_constant", Tf_))
    Tf_ = 0.1;

  controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(controls_topic, 10, &Simulator::controlsCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(virtual_pose_topic, 10);
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(virtual_velocity_topic, 10);
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
  pose_tf_.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, pose_.theta));
  pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), "world", "virtual_robot"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");
  Simulator S;
  return 0;
}
