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

#include "../include/data_recorder.h"

using namespace mtracker;

DataRecorder::DataRecorder() : nh_(""), nh_local_("~"), recording_data_(false), start_mark_(ros::Time::now()) {
  initialize();

  ROS_INFO("MTracker data recorder [OK]");

  ros::Rate rate(loop_rate_);
  while (nh_.ok()) {
    ros::spinOnce();

    if (recording_data_) {
      addLatestData();
    }

    rate.sleep();
  }
}

DataRecorder::~DataRecorder() {
  if (recording_data_) {
    if (use_yaml_)
      emitYamlFile();
    if (use_txt_)
      emitTxtFile();

    recording_data_ = false;
  }
}

void DataRecorder::initialize() {
  if (!nh_.getParam("loop_rate", loop_rate_))
    loop_rate_ = 100;

  std::string pose_topic;
  if (!nh_.getParam("pose_topic", pose_topic))
    pose_topic = "pose";

  std::string controls_topic;
  if (!nh_.getParam("controls_topic", controls_topic))
    controls_topic = "controls";

  std::string scaled_controls_topic;
  if (!nh_.getParam("scaled_controls_topic", scaled_controls_topic))
    scaled_controls_topic = "scaled_controls";

  if (!nh_local_.getParam("use_yaml", use_yaml_))
    use_yaml_ = true;

  if (!nh_local_.getParam("use_txt", use_txt_))
    use_txt_ = true;

  pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(pose_topic, 10, &DataRecorder::poseCallback, this);
  controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(controls_topic, 10, &DataRecorder::controlsCallback, this);
  scaled_controls_sub_ = nh_.subscribe<geometry_msgs::Twist>(scaled_controls_topic, 10, &DataRecorder::scaledControlsCallback, this);
  trigger_srv_ = nh_.advertiseService("data_recorder_trigger_srv", &DataRecorder::trigger, this);
  params_srv_ = nh_.advertiseService("data_recorder_params_srv", &DataRecorder::updateParams, this);

  std::string username = getenv("USER");
  std::string folder = "/home/" + username + "/MTrackerRecords/";
  boost::filesystem::create_directories(folder);
}

void DataRecorder::addLatestData() {
  double t = (start_mark_ - ros::Time::now()).toSec();
  t_.push_back(t);
  pose_list_.push_back(pose_);
  controls_list_.push_back(controls_);
  scaled_controls_list_.push_back(scaled_controls_);
}

void DataRecorder::emitYamlFile() {
/*  static int file_number = 0;
  ++file_number;
  YAML::Emitter emitter;

  time_t actual_time = time(nullptr);
  char header_info[100];
  strftime(header_info, 100, "This data was recorded on %F at %T.", gmtime(&actual_time));

  emitter << YAML::Comment(header_info);
  emitter << YAML::BeginSeq;
  emitter << YAML::Anchor("Pose");
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::Key << "y" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::Key << "theta" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::EndMap;
  emitter << YAML::Anchor("Velocity");
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "v" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::Key << "w" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::EndMap;
  emitter << YAML::Anchor("Time");
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "t" << YAML::Value << YAML::Flow << t_;
    emitter << YAML::EndMap;
  emitter << YAML::EndSeq;

  std::string username = getenv("USER");
  std::string filename = "/home/" + username + "/MTrackerRecords/MTrackerRecord_" + std::to_string(file_number) + ".yaml";
  std::ofstream file(filename);

  file << emitter.c_str();
  file.close();*/
}

void DataRecorder::emitTxtFile() {
  static int file_number = 0;
  ++file_number;

  std::string username = getenv("USER");
  std::string filename = "/home/" + username + "/MTrackerRecords/MTrackerRecord_" + std::to_string(file_number) + ".txt";
  std::ofstream file(filename);

  file << "n \t t \t x \t y \t theta \t v \t w \t v_s \t w_s \n";
  for (int i = 0; i < t_.size(); ++i) {
    file << i << "\t" << t_[i] << "\t" << pose_list_[i].x << "\t" << pose_list_[i].y << "\t" << pose_list_[i].theta
         << "\t" << controls_list_[i].linear.x << "\t" << controls_list_[i].angular.z
         << "\t" << scaled_controls_list_[i].linear.x << "\t" << scaled_controls_list_[i].angular.z << "\n";
  }

  file.close();
}

void DataRecorder::poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  pose_ = *pose_msg;
}

void DataRecorder::controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  controls_ = *controls_msg;
}

void DataRecorder::scaledControlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg) {
  scaled_controls_ = *controls_msg;
}

bool DataRecorder::trigger(mtracker::Trigger::Request& req, mtracker::Trigger::Response& res) {
  if (req.activate && !recording_data_) {
    t_.clear();
    pose_list_.clear();
    controls_list_.clear();
    scaled_controls_list_.clear();

    recording_data_ = true;
    start_mark_ = ros::Time::now();
  }
  else if (!req.activate && recording_data_) {
    recording_data_ = false;

    if (use_txt_)
      emitTxtFile();
    if (use_yaml_)
      emitYamlFile();
  }

  return true;
}

bool DataRecorder::updateParams(mtracker::Params::Request& req, mtracker::Params::Response& res) {
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_recorder");
  mtracker::DataRecorder dr;
  return 0;
}
