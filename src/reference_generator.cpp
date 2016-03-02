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

ReferenceGenerator::ReferenceGenerator() : nh_(""), nh_local_("~"), reference_generator_active_(false) {
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

      publishAll();
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

  if (!nh_.getParam("reference_pose_topic", reference_pose_topic_))
    reference_pose_topic_ = "reference_pose";

  if (!nh_.getParam("reference_velocity_topic", reference_velocity_topic_))
    reference_velocity_topic_ = "reference_velocity";

  if (!nh_.getParam("world_frame", world_frame_))
    world_frame_ = "world";

  if (!nh_local_.getParam("child_frame", child_frame_))
    child_frame_ = "reference";

  int trajectory_type;
  if (!nh_local_.getParam("trajectory_type", trajectory_type))
    trajectory_type = 0;

  if (!nh_local_.getParam("trajectory_paused", paused_))
    paused_ = false;

  switch (trajectory_type) {
    case 0:
      trajectory_ = new Trajectory(0.0, 0.0, 0.0); break;
    case 1:
      trajectory_ = new LinearTrajectory(0.0, 0.1); break;
    case 2:
      trajectory_ = new HarmonicTrajectory(5.0, 1.0, 1.0, 1, 2); break;
    case 3:
      trajectory_ = new LemniscateTrajectory(5.0, 1.0, 1.0, 0.9, 1.0); break;
    default:
      trajectory_ = new Trajectory(0.0, 0.0, 0.0); break;
  }

  time_ =   0.0;
  update(0.0);

  trigger_srv_ = nh_.advertiseService("reference_generator_trigger_srv", &ReferenceGenerator::trigger, this);
  params_srv_ = nh_.advertiseService("reference_generator_params_srv", &ReferenceGenerator::updateParams, this);
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
  double prev_theta = pose_.theta;
  double prev_theta_aux = atan2(sin(pose_.theta), cos(pose_.theta));

  time_ += dt;
  pose_ = trajectory_->calculatePose(time_);
  velocity_ = trajectory_->calculateVelocity(time_);

  double new_theta_aux = atan2(sin(pose_.theta), cos(pose_.theta));
  double theta_diff = new_theta_aux - prev_theta_aux;

  if (theta_diff < -M_PI)
    pose_.theta = prev_theta + theta_diff + 2.0 * M_PI;
  else if (theta_diff > M_PI)
    pose_.theta = prev_theta + theta_diff - 2.0 * M_PI;
  else
    pose_.theta = prev_theta + theta_diff;
}

void ReferenceGenerator::publishAll() {
  pose_pub_.publish(pose_);
  velocity_pub_.publish(velocity_);

  tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
  tf_.setRotation(tf::createQuaternionFromYaw(pose_.theta));
  tf_br_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), world_frame_, child_frame_));

  geometry_msgs::PoseStamped pose_s;

  pose_s.header.stamp = ros::Time::now();
  pose_s.header.frame_id = child_frame_;

  pose_s.pose.position.x = pose_.x;
  pose_s.pose.position.y = pose_.y;
  pose_s.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.theta);

  pose_stamped_pub_.publish(pose_s);
}

bool ReferenceGenerator::trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
  reference_generator_active_ = req.activate;

  if (req.activate) {
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(reference_pose_topic_, 5);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(reference_velocity_topic_, 5);
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(reference_pose_topic_ + "_stamped", 5);
  }
  else {
    pose_pub_.shutdown();
    velocity_pub_.shutdown();
    pose_stamped_pub_.shutdown();
  }

  return true;
}

bool ReferenceGenerator::updateParams(mtracker::Params::Request &req, mtracker::Params::Response &res) {
  // The parameters come as follows:
  // [start, pause, update_params, trajectory_type, x, y, theta, v, T, Rx, Ry, nx, ny]
  if (req.params.size() < 13)
    return false;

  if (req.params[2]) {
    if (req.params[3] == 0.0) {         // Point trajectory
      delete trajectory_;
      trajectory_ = new Trajectory(req.params[4], req.params[5], req.params[6]);
    }
    else if (req.params[3] == 1.0) {    // Linear trajectory
      delete trajectory_;
      trajectory_ = new LinearTrajectory(req.params[4], req.params[5], req.params[6], req.params[7]);
    }
    else if (req.params[3] == 2.0) {    // Harmonic trajectory
      delete trajectory_;
      trajectory_ = new HarmonicTrajectory(req.params[4], req.params[5], req.params[8], req.params[9],
          req.params[10], req.params[11], req.params[12]);
    }
    else if (req.params[3] == 3.0) {    // Lemniscate trajectory
      delete trajectory_;
      trajectory_ = new LemniscateTrajectory(req.params[4], req.params[5], req.params[8], req.params[9],
          req.params[10], req.params[11], req.params[12]);
    }

    time_ = 0.0;
    update(0.0);
  }

  if (req.params[0])
    start();
  else if (req.params[1])
    pause();
  else
    stop();

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "reference_generator");
  ReferenceGenerator rg;
  return 0;
}
