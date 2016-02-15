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
 * Author: Mateusz Przybyla and Wojciech Kowalczyk
 */

#include "../include/obstacle_controller.h"

using namespace mtracker;

ObstacleController::ObstacleController() : nh_(""), nh_local_("~") {
  initialize();

  ROS_INFO("Obstacle controller [OK]");

  ros::Rate rate(loop_rate_);

  while (nh_.ok()) {
    ros::spinOnce();

    computeControls();
    controls_pub_.publish(controls_);

    rate.sleep();
  }
}

void ObstacleController::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  std::string pose_topic;
  if (!nh_.getParam("pose_topic", pose_topic))
    pose_topic = "pose";

  std::string velocity_topic;
  if (!nh_.getParam("velocity_topic", velocity_topic))
    velocity_topic = "velocity";

  std::string controls_topic;
  if (!nh_.getParam("controls_topic", controls_topic))
    controls_topic = "controls";

  pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(pose_topic, 10, &ObstacleController::poseCallback, this);
  velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>(velocity_topic, 10, &ObstacleController::velocityCallback, this);
  obstacles_sub_ = nh_.subscribe<obstacle_detector::Obstacles>("obstacles", 10, &ObstacleController::obstaclesCallback, this);
  controls_pub_ = nh_.advertise<geometry_msgs::Twist>(controls_topic, 10);
  trigger_srv_ = nh_.advertiseService("automatic_controller_trigger_srv", &ObstacleController::trigger, this);
}

void ObstacleController::computeControls() {
  double x = pose_.x;
  double y = pose_.y;
  double theta = pose_.theta;

  double v = 0.0; // Linear velocity
  double w = 0.0; // Angular velocity

  /*
   * HERE PUT THE CODE
   */

  controls_.linear.x = v;
  controls_.angular.z = w;
}

void ObstacleController::poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  pose_ = *pose_msg;
}

void ObstacleController::velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg) {
  velocity_ = *velocity_msg;
}

void ObstacleController::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg) {
  //
}

bool ObstacleController::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  obstacle_controller_active_ = req.activate;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_controller");
  ObstacleController oc;
  return 0;
}
