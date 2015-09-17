/***
 * This node provides the user with an automatic controller
 * for semi-autonomous motion of MTracker. It requires the
 * position of the robot as a geometry_msgs/Pose2D message
 * being published under topic pose. Additionally it can
 * make use of the information from velocity topic.

 * It also needs the reference trajectory published as a
 * geometry_msgs/Pose2D message under topic reference_pose.
 * It can also use the reference_velocity information.
 * As a result it publishes control signals under topic
 * controls.

 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

namespace mtracker
{

class AutoController
{
public:
  AutoController() : nh_(""), nh_params_("~"), autocontroller_switched_on_(false)
  {
    controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 10);
    pose_sub_ = nh_.subscribe("pose", 10, &AutoController::poseCallback, this);
    velocity_sub_ = nh_.subscribe("velocity", 10, &AutoController::velocityCallback, this);
    ref_pose_sub_ = nh_.subscribe("reference_pose", 10, &AutoController::refPoseCallback, this);
    ref_velocity_sub_ = nh_.subscribe("reference_velocity", 10, &AutoController::refVelocityCallback, this);
    trigger_srv_ = nh_.advertiseService("autocontroller_trigger", &AutoController::triggerService, this);
    params_tim_ = nh_.createTimer(ros::Duration(0.25), &AutoController::updateParameters, this, false, false);

    updateParameters(ros::TimerEvent());

    params_tim_.start();
    ROS_INFO("MTracker automatic controller start");
  }

  ~AutoController() {}

  void computeControls()
  {

    /*
     * HERE PUT THE CODE
     */

    controls_.linear.x = 0.0;
    controls_.angular.z = 0.0;
  }

  void publishControls() { controls_pub_.publish(controls_); }
  bool isSwitchedOn() const { return autocontroller_switched_on_; }
  int getLoopRate() const { return loop_rate_; }

private:
  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg)
  { pose_ = *pose_msg; }

  void velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg)
  { velocity_ = *velocity_msg; }

  void refPoseCallback(const geometry_msgs::Pose2D::ConstPtr& ref_pose_msg)
  { ref_pose_ = *ref_pose_msg; }

  void refVelocityCallback(const geometry_msgs::Twist::ConstPtr& ref_velocity_msg)
  { ref_velocity_ = *ref_velocity_msg; }

  bool triggerService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    autocontroller_switched_on_ = !autocontroller_switched_on_;
    return true;
  }

  void updateParameters(const ros::TimerEvent& e)
  {
    if (!nh_.getParam("loop_rate", loop_rate_))
    {
      loop_rate_ = 100;
      nh_.setParam("loop_rate", loop_rate_);
    }
  }

  int loop_rate_;
  bool autocontroller_switched_on_;

  geometry_msgs::Pose2D pose_, ref_pose_;
  geometry_msgs::Twist velocity_, ref_velocity_;
  geometry_msgs::Twist controls_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_params_;

  ros::Publisher controls_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber ref_pose_sub_;
  ros::Subscriber ref_velocity_sub_;
  ros::ServiceServer trigger_srv_;
  ros::Timer params_tim_;
};

} // end namespace mtracker


int main(int argc, char** argv)
{
  ros::init(argc, argv, "automatic_controller");  
  mtracker::AutoController ac;

  ros::Rate rate(ac.getLoopRate());
  while (ros::ok())
  {
    ros::spinOnce();

    if (ac.isSwitchedOn())
    {
      ac.computeControls();
      ac.publishControls();
    }

    rate.sleep();
  }

  return 0;
}
