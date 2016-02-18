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

#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <mtracker/Trigger.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

namespace mtracker {

class DataRecorder
{
public:
  DataRecorder();
  ~DataRecorder();

private:
  void initialize();
  void emitYamlFile();
  void emitTxtFile();

  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose);
  void controlsCallback(const geometry_msgs::Twist::ConstPtr& controls);
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles);
  bool trigger(mtracker::Trigger::Request& req, mtracker::Trigger::Response& res);

  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber pose_sub_;
  ros::Subscriber controls_sub_;
  ros::Subscriber obstacles_sub_;
  ros::ServiceServer trigger_srv_;

  // Recorder variables
  ros::Time start_mark_;

  geometry_msgs::Pose2D latest_pose_;
  geometry_msgs::Twist latest_controls_;
  obstacle_detector::Obstacles latest_obstacles_;

  std::vector<double> t_;
  std::vector<geometry_msgs::Pose2D> pose_list_;
  std::vector<geometry_msgs::Twist> controls_list_;

  bool use_yaml_;
  bool use_txt_;

  int loop_rate_;
  bool recording_data_;
};

} // namespace mtracker

#endif // DATA_RECORDER_H
