#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>

/***
  This node provides the user with a manual controller
  for teleoperation purposes. It requires messages of
  type sensors_msgs/Joy being published under topic /joy
  in order to work. It translates the joystick values
  into the proper control signals, which are further
  published under topic /controls. It works in an
  asynchronous manner, meaning that the control signals
  are published only after the new joystick signals
  arrive.

  Mateusz Przybyla
  Chair of Control and Systems Engineering
  Faculty of Computing
  Poznan University of Technology
***/


void shutdown(int sig)
{
  ROS_INFO("MTracker manual controller shutdown");
  ros::shutdown();
}


class ManualController {
public:
  ros::Publisher  ctrl_pub;
  ros::Subscriber joy_sub;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    geometry_msgs::Twist controls;

    // TODO: Think about making the gains somehow changeable
    controls.linear.x  = 0.5 * joy_msg->axes[1];
    controls.angular.z = 1.0 * joy_msg->axes[0];

    ctrl_pub.publish(controls);
  }
};


int main(int argc, char **argv)
{
  ManualController mc;

  ros::init(argc, argv, "manual_controller");
  ros::NodeHandle n;

  ROS_INFO("MTracker manual controller start");

  mc.ctrl_pub = n.advertise<geometry_msgs::Twist>("/controls", 10);
  mc.joy_sub = n.subscribe("/joy", 10, &ManualController::joyCallback, &mc);

  signal(SIGINT, shutdown);

  ros::spin();

  return 0;
}
