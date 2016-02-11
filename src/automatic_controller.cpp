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

#include "../include/automatic_controller.h"

using namespace mtracker;

AutomaticController::AutomaticController() : nh_(""), nh_local_("~") {
  initialize();

  ROS_INFO("Automatic controller [OK]");

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    computeControls();
    controls_pub_.publish(controls_);

    rate.sleep();
  }
}

void AutomaticController::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  std::string pose_topic;
  if (!nh_.getParam("pose_topic", pose_topic))
    pose_topic = "pose";

  std::string velocity_topic;
  if (!nh_.getParam("velocity_topic", velocity_topic))
    velocity_topic = "velocity";

  std::string reference_pose_topic;
  if (!nh_.getParam("reference_pose_topic", reference_pose_topic))
    reference_pose_topic = "reference_pose";

  std::string reference_velocity_topic;
  if (!nh_.getParam("reference_velocity_topic", reference_velocity_topic))
    reference_velocity_topic = "reference_velocity";

  std::string controls_topic;
  if (!nh_.getParam("controls_topic", controls_topic))
    controls_topic = "controls";

  pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(pose_topic, 10, &AutomaticController::poseCallback, this);
  velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>(velocity_topic, 10, &AutomaticController::velocityCallback, this);
  ref_pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(reference_pose_topic, 10, &AutomaticController::refPoseCallback, this);
  ref_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>(reference_velocity_topic, 10, &AutomaticController::refVelocityCallback, this);
  controls_pub_ = nh_.advertise<geometry_msgs::Twist>(controls_topic, 10);
}

void AutomaticController::computeControls() {
  double x = pose_.x;
  double y = pose_.y;
  double theta = pose_.theta;

  double x_d = ref_pose_.x;
  double y_d = ref_pose_.y;
  double theta_d = ref_pose_.theta;

  double v = 0.0; // Linear velocity
  double w = 0.0; // Angular velocity

  /*
   * HERE PUT THE CODE
   */

  controls_.linear.x = v;
  controls_.angular.z = w;
}

void AutomaticController::poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  pose_ = *pose_msg;
}

void AutomaticController::velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg) {
  velocity_ = *velocity_msg;
}

void AutomaticController::refPoseCallback(const geometry_msgs::Pose2D::ConstPtr& ref_pose_msg) {
  ref_pose_ = *ref_pose_msg;
}

void AutomaticController::refVelocityCallback(const geometry_msgs::Twist::ConstPtr& ref_velocity_msg) {
  ref_velocity_ = *ref_velocity_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "automatic_controller");
  AutomaticController ac;
  return 0;
}
