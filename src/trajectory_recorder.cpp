#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <string>
#include <ctime>

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

struct Recorder
{
  std::vector<double> x, y, theta;
  std::vector<double> v, w;

  YAML::Emitter yaml;

  bool recording_data;
  int  file_number;

  Recorder() : file_number(1), recording_data(false) {}

  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg)
  {
    if (recording_data)
    {
      x.push_back(pose_msg->x);
      y.push_back(pose_msg->y);
      theta.push_back(pose_msg->theta);
    }
  }

  void velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg)
  {
    if (recording_data)
    {
      v.push_back(velocity_msg->linear.x);
      w.push_back(velocity_msg->angular.z);
    }
  }

  bool triggerRecording(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if (recording_data)
    {
      emitYamlFile();

      v.clear();
      w.clear();
      x.clear();
      y.clear();
      theta.clear();
      file_number++;
    }

    recording_data = ~recording_data;

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
      emitter << YAML::Key << "x" << YAML::Value << YAML::Flow << x;
      emitter << YAML::Key << "y" << YAML::Value << YAML::Flow << y;
      emitter << YAML::Key << "theta" << YAML::Value << YAML::Flow << theta;
      emitter << YAML::EndMap;
    emitter << YAML::Anchor("Velocity");
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "v" << YAML::Value << YAML::Flow << v;
      emitter << YAML::Key << "w" << YAML::Value << YAML::Flow << w;
      emitter << YAML::EndMap;
    emitter << YAML::Anchor("Parameters");
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "Tp" << YAML::Value << 0.01;
      emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    std::string username = getenv("USER");
    std::cout << username << std::endl;
    std::string filename = "/home/" + username + "/MTrackerRecords/MTrackerRecord_" + std::to_string(file_number) + ".yaml";
    std::ofstream file(filename);

    file << emitter.c_str();
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_recorder");
  ros::NodeHandle n;

  Recorder r;

  ros::Subscriber pose_sub = n.subscribe<geometry_msgs::Pose2D>("pose", 10, &Recorder::poseCallback, &r);
  ros::Subscriber velocity_sub = n.subscribe<geometry_msgs::Twist>("velocity", 10, &Recorder::velocityCallback, &r);
  ros::ServiceServer trig_srv = n.advertiseService("trajectory_recorder", &Recorder::triggerRecording, &r);

  ROS_INFO("MTracker trajectory recorder start");

  ros::spin();

  return 0;
}
