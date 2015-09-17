/***
 * This node simulates the behavior of a real MTracker
 * robot. It receives the `controls` signals and publish
 * the virtual odometry information as `virtual_pose` and
 * `virtual_velocity`. It also broadcasts the tf frame
 * `virtual_robot` which is represented w.r.t. frame `world`.
 * The node works in synchronous manner with the default
 * rate of 100 Hz.
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

class Simulator
{
public:
  Simulator() : nh_(""), nh_params_("~"), simulator_switched_on_(false)
  {
    controls_sub_ = nh_.subscribe("controls", 10, &Simulator::controlsCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("virtual_pose", 10);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("virtual_velocity", 10);
    trigger_srv_ = nh_.advertiseService("simulator_trigger", &Simulator::triggerService, this);
    params_tim_ = nh_.createTimer(ros::Duration(0.25), &Simulator::updateParams, this, false, false);

    updateParams(ros::TimerEvent());

    params_tim_.start();
    ROS_INFO("MTracker simulator start");
  }

  ~Simulator() {}

  void computeVelocity()
  {
    velocity_.linear.x  += Tp_ / (Tp_ + Tf_) * (controls_.linear.x - velocity_.linear.x);
    velocity_.angular.z += Tp_ / (Tp_ + Tf_) * (controls_.angular.z - velocity_.angular.z);
  }

  void computePose()
  {
    pose_.x += Tp_ * velocity_.linear.x * cos(pose_.theta);
    pose_.y += Tp_ * velocity_.linear.x * sin(pose_.theta);
    pose_.theta += Tp_ * velocity_.angular.z;
  }

  void publishVelocity() { velocity_pub_.publish(velocity_); }
  void publishPose() { pose_pub_.publish(pose_); }
  void publishTransform()
  {
    pose_tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
    pose_tf_.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, pose_.theta));
    pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), "world", "virtual_robot"));
  }
  int getLoopRate() const { return loop_rate_; }
  bool isSwitchedOn() const { return simulator_switched_on_; }

private:
  void controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg)
  { controls_ = *controls_msg; }

  bool triggerService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    simulator_switched_on_ = !simulator_switched_on_;
    return true;
  }

  void updateParams(const ros::TimerEvent& e)
  {
    if (!nh_.getParam("loop_rate", loop_rate_))
    {
      loop_rate_ = 100;
      nh_.setParam("loop_rate", loop_rate_);
    }
    Tp_ = 1.0 / loop_rate_;

    if (!nh_params_.getParam("robot_inertia", Tf_))
      Tf_ = 0.1;
  }

  int loop_rate_;
  double Tp_;
  double Tf_;
  bool simulator_switched_on_;

  geometry_msgs::Pose2D pose_;
  geometry_msgs::Twist velocity_;
  geometry_msgs::Twist controls_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_params_;

  ros::Subscriber controls_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::ServiceServer trigger_srv_;
  ros::Timer params_tim_;

  tf::TransformBroadcaster pose_bc_;
  tf::Transform pose_tf_;
};

} // end namespace mtracker


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtracker_simulator");
  mtracker::Simulator s;

  ros::Rate rate(s.getLoopRate());
  while (ros::ok())
  {
    ros::spinOnce();

    if (s.isSwitchedOn())
    {
      s.computeVelocity();
      s.computePose();
      s.publishVelocity();
      s.publishPose();
      s.publishTransform();
    }

    rate.sleep();
  }

  return 0;
}
