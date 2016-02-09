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

#include "../include/manual_controller.h"

using namespace mtracker;

ManualController::ManualController() : nh_(""), nh_local_("~") {
  initialize();

  ROS_INFO("MTracker manual controller [OK]");

  ros::spin();
}

void ManualController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
  controls_.linear.x = v_gain_ * joy_msg->axes[1];
  controls_.angular.z = w_gain_ * joy_msg->axes[0];
  controls_pub_.publish(controls_);
}

void ManualController::keysCallback(const geometry_msgs::Twist::ConstPtr& keys_msg) {
  controls_.linear.x = v_gain_ * keys_msg->linear.x;
  controls_.angular.z = w_gain_ * keys_msg->angular.z;
  controls_pub_.publish(controls_);
}

void ManualController::initialize() {
  std::string controls_topic;
  if (!nh_.getParam("controls_topic", controls_topic))
    controls_topic = "controls";

  std::string joy_topic;
  if (!nh_.getParam("joy_topic", joy_topic))
    joy_topic = "joy";

  std::string keys_topic;
  if (!nh_.getParam("keys_topic", keys_topic))
    keys_topic = "keys";

  if (!nh_local_.getParam("linear_gain", v_gain_))
    v_gain_ = 0.4;

  if (!nh_local_.getParam("angular_gain", w_gain_))
    w_gain_ = 1.5;

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic, 10, &ManualController::joyCallback, this);
  keys_sub_ = nh_.subscribe<geometry_msgs::Twist>(keys_topic, 10, &ManualController::keysCallback, this);
  controls_pub_ = nh_.advertise<geometry_msgs::Twist>(controls_topic, 10);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "manual_controller");
  ManualController mc;
  return 0;
}

