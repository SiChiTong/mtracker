#ifndef TRAJECTORY_RECORDER_H
#define TRAJECTORY_RECORDER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <ctime>

namespace mtracker {

class Recorder
{
public:
  Recorder();
  ~Recorder();

private:
  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose);
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg);
  bool triggerRecording(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void updateParams(const ros::TimerEvent& event);

  void emitYamlFile();
  void emitTxtFile();

  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::ServiceServer trigger_srv_;
  ros::Timer params_tim_;

  // Recorder variables
  bool recording_data_;
  ros::Time start_mark_;

  std::vector<double> t_;
  std::vector<double> x_, y_, theta_;
  std::vector<double> v_, w_;

  // Parameters
  std::string p_pose_topic_;
  std::string p_velocity_topic_;
  bool p_use_yaml_;
  bool p_use_txt_;
};

} // namespace mtracker

#endif // TRAJECTORY_RECORDER_H
