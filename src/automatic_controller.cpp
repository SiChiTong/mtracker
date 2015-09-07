#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

/***
 * This node provides the user with an automatic controller
 * for semi-autonomous motion of MTracker. It requires the
 * position of the robot as a geometry_msgs/Pose2D message
 * being published under topic /pose. Additionally it can
 * make use of the information from /velocity topic.

 * It also needs the reference trajectory published as a
 * geometry_msgs/Pose2D message under topic /reference_pose.
 * It can also use the /reference_velocity information.
 * As a result it publishes control signals under topic
 * /controls.

 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/

class AutomaticController {
public:
  ros::Publisher  ctrl_pub;
  ros::Subscriber pos_sub;
  ros::Subscriber vel_sub;
  ros::Subscriber ref_pos_sub;
  ros::Subscriber ref_vel_sub;

  geometry_msgs::Pose2D pose, ref_pose;
  geometry_msgs::Twist  velocity, ref_velocity;
  geometry_msgs::Twist  controls;

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

  void publishControls()
  {
    ctrl_pub.publish(controls);
  }
};


int main(int argc, char **argv)
{
  AutomaticController ac;

  ros::init(argc, argv, "automatic_controller");
  ros::NodeHandle n;

  ac.ctrl_pub = n.advertise<geometry_msgs::Twist>("/controls", 10);
  ac.pos_sub  = n.subscribe("/pose", 10, &AutomaticController::poseCallback, &ac);
  ac.vel_sub  = n.subscribe("/velocity", 10, &AutomaticController::velocityCallback, &ac);
  ac.ref_pos_sub = n.subscribe("/reference_pose", 10, &AutomaticController::refPoseCallback, &ac);
  ac.ref_vel_sub = n.subscribe("/reference_velocity", 10, &AutomaticController::refVelocityCallback, &ac);

  ROS_INFO("MTracker automatic controller start");

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();

    ac.computeControls();
    ac.publishControls();

    rate.sleep();
  }

  return 0;
}
