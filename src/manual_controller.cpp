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

void ManualController::initialize() {
  if (!nh_.getParam("controls_topic", controls_topic_))
    controls_topic_ = "controls";

  if (!nh_.getParam("joy_topic", joy_topic_))
    joy_topic_ = "joy";

  if (!nh_.getParam("keys_topic", keys_topic_))
    keys_topic_ = "keys";

  if (!nh_local_.getParam("use_joy", use_joy_))
    use_joy_ = true;

  if (!nh_local_.getParam("use_keys", use_keys_))
    use_keys_ = true;

  if (!nh_local_.getParam("k_v", k_v_))
    k_v_ = 0.6;

  if (!nh_local_.getParam("k_w", k_w_))
    k_w_ = 1.5;

  trigger_srv_ = nh_.advertiseService("manual_controller_trigger_srv", &ManualController::trigger, this);
  params_srv_ = nh_.advertiseService("manual_controller_params_srv", &ManualController::updateParams, this);
}

void ManualController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
  if (use_joy_) {
    controls_.linear.x = k_v_ * joy_msg->axes[1];
    controls_.angular.z = k_w_ * joy_msg->axes[0];
    controls_pub_.publish(controls_);
  }
}

void ManualController::keysCallback(const geometry_msgs::Twist::ConstPtr& keys_msg) {
  if (use_keys_) {
    controls_.linear.x = k_v_ * keys_msg->linear.x;
    controls_.angular.z = k_w_ * keys_msg->angular.z;
    controls_pub_.publish(controls_);
  }
}

bool ManualController::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  if (req.activate) {
    controls_pub_ = nh_.advertise<geometry_msgs::Twist>(controls_topic_, 5);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic_, 5, &ManualController::joyCallback, this);
    keys_sub_ = nh_.subscribe<geometry_msgs::Twist>(keys_topic_, 5, &ManualController::keysCallback, this);
  }
  else {
    controls_.linear.x = 0.0;
    controls_.angular.z = 0.0;
    controls_pub_.publish(controls_);

    controls_pub_.shutdown();
    joy_sub_.shutdown();
    keys_sub_.shutdown();
  }

  return true;
}

bool ManualController::updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res) {
  if (req.params.size() >= 4) {
    k_v_ = req.params[0];
    k_w_ = req.params[1];
    use_joy_ = static_cast<bool>(req.params[2]);
    use_keys_ = static_cast<bool>(req.params[3]);

    return true;
  }
  else
    return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_controller");
  ManualController mc;
  return 0;
}

