#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

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

struct AutomaticController
{
  geometry_msgs::Pose2D pose, ref_pose;
  geometry_msgs::Twist  velocity, ref_velocity;
  geometry_msgs::Twist  controls;

  // Parameters
  int loop_rate;

  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pos_msg)
  {
    pose = *pos_msg;
    pose.theta = atan2(sin(pose.theta), cos(pose.theta));
  }

  void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel_msg)
  {
    velocity = *vel_msg;
  }

  void refPoseCallback(const geometry_msgs::Pose2D::ConstPtr& ref_pos_msg)
  {
    ref_pose = *ref_pos_msg;
    ref_pose.theta = atan2(sin(ref_pose.theta), cos(ref_pose.theta));
  }

  void refVelocityCallback(const geometry_msgs::Twist::ConstPtr& ref_vel_msg)
  {
    ref_velocity = *ref_vel_msg;
  }

  void computeControls()
  { 
    // HERE PUT THE CODE

    controls.linear.x = 0.0;
    controls.angular.z = 0.0;
  }

  void updateParameters(const ros::TimerEvent& event)
  {
    ros::NodeHandle nh_global;

    if (!nh_global.getParam("loop_rate", loop_rate))
    {
      loop_rate = 100;
      nh_global.setParam("loop_rate", loop_rate);
    }
  }
};


int main(int argc, char** argv)
{
  AutomaticController ac;

  ros::init(argc, argv, "automatic_controller");
  ros::NodeHandle n;

  {
    ros::TimerEvent e;
    ac.updateParameters(e);
  }

  ros::Publisher  ctrl_pub = n.advertise<geometry_msgs::Twist>("controls", 10);
  ros::Subscriber pos_sub = n.subscribe("pose", 10, &AutomaticController::poseCallback, &ac);
  ros::Subscriber vel_sub = n.subscribe("velocity", 10, &AutomaticController::velocityCallback, &ac);
  ros::Subscriber ref_pos_sub = n.subscribe("reference_pose", 10, &AutomaticController::refPoseCallback, &ac);
  ros::Subscriber ref_vel_sub = n.subscribe("reference_velocity", 10, &AutomaticController::refVelocityCallback, &ac);
  ros::Timer      params_tim = n.createTimer(ros::Duration(0.25), &AutomaticController::updateParameters, &ac);

  ROS_INFO("MTracker automatic controller start");

  ros::Rate rate(ac.loop_rate);

  while (ros::ok())
  {
    ros::spinOnce();

    ac.computeControls();
    ctrl_pub.publish(ac.controls);

    rate.sleep();
  }

  return 0;
}
