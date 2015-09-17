/***
 * This node is used for trajectory recording. When triggered on,
 * it synchronously saves consecutive Poses and Velocities obtained
 * from the robot or its simulation. It can be further processed as
 * reference trajectory for some other robot. The data is stored in
 * a .yaml file. To trigger data recording on, a ros service call
 * must be made for the "trajectory_recorder" service.
 *
 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <ctime>

namespace mtracker
{

class Recorder
{
public:
  Recorder() : nh_(""), nh_params_("~"), file_number_(1), recording_data_(false)
  {
    pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("pose", 10, &Recorder::poseCallback, this);
    velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("velocity", 10, &Recorder::velocityCallback, this);
    trigger_srv_ = nh_.advertiseService("trigger_recording", &Recorder::triggerRecording, this);

    ROS_INFO("MTracker trajectory recorder start");
  }

  ~Recorder() {}

private:
  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg)
  {
    if (recording_data_)
    {
      x_.push_back(pose_msg->x);
      y_.push_back(pose_msg->y);
      theta_.push_back(pose_msg->theta);
    }
  }

  void velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg)
  {
    if (recording_data_)
    {
      v_.push_back(velocity_msg->linear.x);
      w_.push_back(velocity_msg->angular.z);
    }
  }

  bool triggerRecording(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if (recording_data_)
    {
      emitYamlFile();

      v_.clear();
      w_.clear();
      x_.clear();
      y_.clear();
      theta_.clear();
      file_number_++;
    }

    recording_data_ = ~recording_data_;
    return true;
  }

  void emitYamlFile()
  {
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
    emitter << YAML::Anchor("Parameters");
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "Tp" << YAML::Value << 0.01;
      emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    std::string username = getenv("USER");
    std::cout << username << std::endl;
    std::string filename = "/home/" + username + "/MTrackerRecords/MTrackerRecord_" + std::to_string(file_number_) + ".yaml";
    std::ofstream file(filename);

    file << emitter.c_str();
  }

  bool recording_data_;
  int  file_number_;

  std::vector<double> x_, y_, theta_;
  std::vector<double> v_, w_;

  YAML::Emitter yaml_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_params_;

  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::ServiceServer trigger_srv_;
};

} // end namespace mtracker


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_recorder");
  mtracker::Recorder r;
  ros::spin();

  return 0;
}
