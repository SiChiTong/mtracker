#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

/*** 
 * This node provides the user with a manual controller
 * for teleoperation purposes. It requires messages of
 * type sensors_msgs/Joy being published under topic /joy
 * in order to work. It translates the joystick values
 * into the proper control signals, which are further
 * published under topic /controls. It works in an
 * asynchronous manner, meaning that the control signals
 * are published only after the new joystick signals
 * arrive.

 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/


struct ManualController
{
  ros::Publisher controls_pub;
  geometry_msgs::Twist controls;

  double k_v;  // Linear velocity gain
  double k_w;  // Angular velocity gain

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    controls.linear.x  = k_v * joy_msg->axes[1];
    controls.angular.z = k_w * joy_msg->axes[0];

    controls_pub.publish(controls);
  }

  void updateParams(const ros::TimerEvent& e)
  {
    static ros::NodeHandle nh_priv("~");
    if (!nh_priv.getParam("v_gain", k_v))
      k_v = 0.2;

    if (!nh_priv.getParam("w_gain", k_w))
      k_w = 0.2;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_controller");
  ros::NodeHandle n;

  ManualController mc;

  {
    ros::TimerEvent e;
    mc.updateParams(e);
  }

  mc.controls_pub = n.advertise<geometry_msgs::Twist>("/controls", 10);
  ros::Subscriber joy_sub  = n.subscribe("/joy", 10, &ManualController::joyCallback, &mc);
  ros::Timer params_tim = n.createTimer(ros::Duration(0.25), &ManualController::updateParams, &mc);


  ROS_INFO("MTracker manual controller start");

  ros::spin();

  return 0;
}
