
#include "../include/trajectory_recorder.h"

namespace mtracker
{

void Recorder::updateParams(const ros::TimerEvent& event) {
  static bool first_call = true;

  if (first_call) {
    if (!nh_local_.getParam("pose_topic", p_pose_topic_))
      p_pose_topic_ = "pose";

    if (!nh_local_.getParam("velocity_topic", p_velocity_topic_))
      p_velocity_topic_ = "velocity";

    first_call = false;
  }

  if (!nh_local_.getParam("use_yaml", p_use_yaml_))
    p_use_yaml_ = true;

  if (!nh_local_.getParam("use_txt", p_use_txt_))
    p_use_txt_ = true;
}

Recorder::Recorder() : nh_(""), nh_local_("~"), recording_data_(false), start_mark_(ros::Time::now()) {
  updateParams(ros::TimerEvent());

  pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(p_pose_topic_, 10, &Recorder::poseCallback, this);
  velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>(p_velocity_topic_, 10, &Recorder::velocityCallback, this);
  trigger_srv_ = nh_.advertiseService("trigger_recording", &Recorder::triggerRecording, this);
  params_tim_ = nh_.createTimer(ros::Duration(0.5), &Recorder::updateParams, this);

  std::string username = getenv("USER");
  std::string folder = "/home/" + username + "/MTrackerRecords/";
  boost::filesystem::create_directories(folder);

  ROS_INFO("MTracker trajectory recorder start");
}

Recorder::~Recorder() {
  if (recording_data_) {
    if (p_use_yaml_)
      emitYamlFile();
    if (p_use_txt_)
      emitTxtFile();

    recording_data_ = false;
  }
}

void Recorder::poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose) {
  if (recording_data_) {
    double t = (start_mark_ - ros::Time::now()).toSec();
    t_.push_back(t);
    x_.push_back(pose->x);
    y_.push_back(pose->y);
    theta_.push_back(pose->theta);
  }
}

void Recorder::velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg) {
  if (recording_data_) {
    v_.push_back(velocity_msg->linear.x);
    w_.push_back(velocity_msg->angular.z);
  }
}

bool Recorder::triggerRecording(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  if (recording_data_) {
    if (p_use_yaml_)
      emitYamlFile();
    if (p_use_txt_)
      emitTxtFile();

    t_.clear();
    x_.clear();
    y_.clear();
    theta_.clear();
    v_.clear();
    w_.clear();
  }

  start_mark_ = ros::Time::now();
  recording_data_ = ~recording_data_;

  return true;
}

void Recorder::emitYamlFile() {
  static int file_number = 0;
  ++file_number;
  YAML::Emitter emitter;

  time_t actual_time = time(nullptr);
  char header_info[100];
  strftime(header_info, 100, "This data was recorded on %F at %T.", gmtime(&actual_time));

  emitter << YAML::Comment(header_info);
  emitter << YAML::BeginSeq;
  emitter << YAML::Anchor("Pose");
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << YAML::Flow << x_;
    emitter << YAML::Key << "y" << YAML::Value << YAML::Flow << y_;
    emitter << YAML::Key << "theta" << YAML::Value << YAML::Flow << theta_;
    emitter << YAML::EndMap;
  emitter << YAML::Anchor("Velocity");
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "v" << YAML::Value << YAML::Flow << v_;
    emitter << YAML::Key << "w" << YAML::Value << YAML::Flow << w_;
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
  file.close();
}

void Recorder::emitTxtFile() {
  static int file_number = 0;
  ++file_number;

  std::string username = getenv("USER");
  std::string filename = "/home/" + username + "/MTrackerRecords/MTrackerRecord_" + std::to_string(file_number) + ".txt";
  std::ofstream file(filename);

  file << "n \t t \t x \t y \t theta \n"; // Velocities: \t v \t w
  for (uint i = 0; i < x_.size(); ++i) {
    file << i << "\t" << t_[i] << "\t" << x_[i] << "\t" << y_[i] << "\t" << theta_[i] << "\n"; // Velocities: "\t" << v_[i] << "\t" << w_[i] <<
  }

  file.close();
}

} // end namespace mtracker


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_recorder");
  mtracker::Recorder r;

  ros::spin();

  return 0;
}
