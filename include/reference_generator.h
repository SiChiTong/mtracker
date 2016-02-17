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

#ifndef REFERENCE_GENERATOR_H
#define REFERENCE_GENERATOR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mtracker/Trigger.h>
#include <mtracker/PlayPause.h>
#include <tf/transform_broadcaster.h>

#include "../include/trajectories.h"

namespace mtracker
{

class ReferenceGenerator
{
public:
   ReferenceGenerator();
  ~ReferenceGenerator();

private:
  void initialize();
  void start();
  void stop();
  void pause();
  void update(double dt);
  void publish();

  bool trigger(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res);
  bool playPause(mtracker::PlayPause::Request &req, mtracker::PlayPause::Response &res);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher pose_stamped_pub_;
  ros::ServiceServer trigger_srv_;
  ros::ServiceServer play_pause_srv_;

  tf::TransformBroadcaster tf_br_;
  tf::StampedTransform tf_;

  std::string world_frame_;
  std::string child_frame_;

  geometry_msgs::Pose2D pose_;
  geometry_msgs::Twist velocity_;

  Trajectory* trajectory_;
  double time_;
  bool paused_;

  int loop_rate_;
  bool reference_generator_active_;
};

} // namespace mtracker

#endif // REFERENCE_GENERATOR_H
