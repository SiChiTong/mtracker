/***
 * This is a time-based reference trajectory generator
 * used for the automatic controller of the MTracker.
 * It calculates the pose and velocity of a reference
 * robot and publishes them under topics reference_pose
 * and reference_velocity, respectively. It also
 * broadcasts the tf frame called reference_robot. The
 * node works synchronously with a rate of 100 Hz.
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
#include <tf/transform_broadcaster.h>

namespace mtracker
{

class RefGenerator
{
public:
  RefGenerator() : nh_(""), nh_params_("~"), refgenerator_switched_on_(false)
  {
    ref_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("reference_pose", 10);
    ref_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("reference_velocity", 10);
    trigger_srv_ = nh_.advertiseService("refgenerator_trigger", &RefGenerator::triggerService, this);
    params_tim_ = nh_.createTimer(ros::Duration(0.25), &RefGenerator::updateParams, this, false, false);

    updateParams(ros::TimerEvent());
    params_tim_.start();
    ROS_INFO("MTracker reference generator start");
  }

  ~RefGenerator() {}

  void calculateRefPose(double t)
  {
    switch (trajectory_)
    {
    case POINT:
      ref_pose_.x = x_o_;
      ref_pose_.y = y_o_;
      ref_pose_.theta = theta_o_;
      break;
    case LINEAR:
      ref_pose_.x = x_o_ + v_ * cos(theta_o_) * t;
      ref_pose_.y = y_o_ + v_ * sin(theta_o_) * t;
      ref_pose_.theta = theta_o_;
      break;
    case CIRCULAR:
      ref_pose_.x = x_o_ + R_ * cos(w_ * t);
      ref_pose_.y = y_o_ + R_ * sin(w_ * t);
      ref_pose_.theta = w_ * t + M_PI_2;
      break;
    }
  }

  void calculateRefVelocity(double t)
  {
    switch (trajectory_)
    {
    case POINT:
      ref_velocity_.linear.x  = 0.0;
      ref_velocity_.linear.y  = 0.0;
      ref_velocity_.angular.z = 0.0;
      break;
    case LINEAR:
      ref_velocity_.linear.x  = v_ * cos(theta_o_);
      ref_velocity_.linear.y  = v_ * sin(theta_o_);
      ref_velocity_.angular.z = 0.0;
      break;
    case CIRCULAR:
      ref_velocity_.linear.x  = -R_ * w_ * sin(w_ * t);
      ref_velocity_.linear.y  =  R_ * w_ * cos(w_ * t);
      ref_velocity_.angular.z =  w_;
      break;
    case POLYNOMIAL:
      break;
    case FROM_FILE:
      readRefFromFile();
      break;
    }
  }

  void publishRefPose() { ref_pose_pub_.publish(ref_pose_); }
  void publishRefVelocity() { ref_velocity_pub_.publish(ref_velocity_); }
  void publishRefTransform()
  {
    ref_pose_tf_.setOrigin(tf::Vector3(ref_pose_.x, ref_pose_.y, 0.0));
    ref_pose_tf_.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, ref_pose_.theta));
    ref_pose_bc_.sendTransform(tf::StampedTransform(ref_pose_tf_, ros::Time::now(), "world", "reference_robot"));
  }
  int getLoopRate() const { return loop_rate_; }
  int isSwitchedOn() const { return refgenerator_switched_on_; }

private:
  enum TrajectoryType { POINT, LINEAR, CIRCULAR, POLYNOMIAL, FROM_FILE };

  bool triggerService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    refgenerator_switched_on_ = !refgenerator_switched_on_;
    return true;
  }

  void updateParams(const ros::TimerEvent& e)
  {
    if (!nh_.getParam("loop_rate", loop_rate_))
    {
      loop_rate_ = 100;
      nh_.setParam("loop_rate", loop_rate_);
    }

    int trajectory_type;
    if (!nh_params_.getParam("trajectory_type", trajectory_type))
      trajectory_type == 0;

    else if (trajectory_type == 1) trajectory_ = LINEAR;
    else if (trajectory_type == 2) trajectory_ = CIRCULAR;
    else if (trajectory_type == 3) trajectory_ = POLYNOMIAL;
    else if (trajectory_type == 4) trajectory_ = FROM_FILE;
    else trajectory_ = POINT;

    if (!nh_params_.getParam("x_origin", x_o_))
      x_o_ = 0.0;
    if (!nh_params_.getParam("y_origin", y_o_))
      y_o_ = 0.0;
    if (!nh_params_.getParam("theta_origin", theta_o_))
      theta_o_ = 0.0;
    if (!nh_params_.getParam("v", v_))
      v_ = 0.0;
    if (!nh_params_.getParam("w", w_))
      w_ = 0.0;
    if (!nh_params_.getParam("radius", R_))
      R_ = 0.0;
    if (!nh_params_.getParam("ref_filename", ref_filename_))
      ref_filename_ = "";
  }

  void readRefFromFile()
  {
    // TODO: load data from yaml file
  }

  TrajectoryType trajectory_;
  std::string ref_filename_;
  double x_o_, y_o_, theta_o_;
  double v_, w_;
  double R_;
  int loop_rate_;
  bool refgenerator_switched_on_;

  geometry_msgs::Pose2D ref_pose_;
  geometry_msgs::Twist  ref_velocity_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_params_;

  ros::Publisher ref_pose_pub_;
  ros::Publisher ref_velocity_pub_;
  ros::ServiceServer trigger_srv_;
  ros::Timer params_tim_;

  tf::TransformBroadcaster ref_pose_bc_;
  tf::Transform ref_pose_tf_;
};

} // end namespace mtracker


int main(int argc, char** argv)
{
  ros::init(argc, argv, "reference_generator");
  mtracker::RefGenerator rg;

  ros::Rate rate(rg.getLoopRate());
  ros::Time tic, toc;
  tic = ros::Time::now();
  while (ros::ok())
  {
    ros::spinOnce();

    if (rg.isSwitchedOn())
    {
      toc = ros::Time::now();
      float t = (toc - tic).toSec();

      rg.calculateRefPose(t);
      rg.calculateRefVelocity(t);
      rg.publishRefPose();
      rg.publishRefVelocity();
      rg.publishRefTransform();
    }

    rate.sleep();
  }

  return 0;
}
