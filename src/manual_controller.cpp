/*** 
 * This node provides the user with a manual controller
 * for teleoperation purposes. It requires messages of
 * type sensors_msgs/Joy being published under topic joy
 * in order to work. It translates the joystick values
 * into the proper control signals, which are further
 * published under topic controls. It works in an
 * asynchronous manner, meaning that the control signals
 * are published only after the new joystick signals
 * arrive.
 *
 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

namespace mtracker
{

class ManController
{
public:
  ManController() : nh_(""), nh_params_("~"), mancontroller_switched_on_(false)
  {
    controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 10);
    joy_sub_ = nh_.subscribe("joy", 10, &ManController::joyCallback, this);
    keys_sub_ = nh_.subscribe("keys", 10, &ManController::keysCallback, this);
    trigger_srv_ = nh_.advertiseService("mancontroller_trigger", &ManController::triggerService, this);
    params_tim_ = nh_.createTimer(ros::Duration(0.25), &ManController::updateParams, this, false, false);

    updateParams(ros::TimerEvent());

    params_tim_.start();
    ROS_INFO("MTracker manual controller start");
  }

  ~ManController() {}

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    if (mancontroller_switched_on_)
    {
      controls_.linear.x  = v_gain_ * joy_msg->axes[1];
      controls_.angular.z = w_gain_ * joy_msg->axes[0];
      controls_pub_.publish(controls_);
    }
  }

  void keysCallback(const geometry_msgs::Twist::ConstPtr& keys_msg)
  {
    if (mancontroller_switched_on_)
    {
      controls_ = *keys_msg;
      controls_pub_.publish(controls_);
    }
  }

  bool triggerService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    ROS_INFO("asd");
    mancontroller_switched_on_ = !mancontroller_switched_on_;
    return true;
  }

  void updateParams(const ros::TimerEvent& e)
  {
    if (!nh_params_.getParam("v_gain", v_gain_))
      v_gain_ = 0.2;

    if (!nh_params_.getParam("w_gain", w_gain_))
      w_gain_ = 0.2;
  }

  double v_gain_;  // Linear velocity gain
  double w_gain_;  // Angular velocity gain
  bool mancontroller_switched_on_;

  geometry_msgs::Twist controls_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_params_;

  ros::Publisher controls_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber keys_sub_;
  ros::ServiceServer trigger_srv_;
  ros::Timer params_tim_;
};

} // end namespace mtracker


int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_controller");
  mtracker::ManController mc;
  ros::spin();

  return 0;
}

