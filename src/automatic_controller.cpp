#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

/***
  This node provides the user with an automatic controller
  for semi-autonomous motion of MTracker. It requires the
  position of the robot as a geometry_msgs/Pose2D message
  being published under topic /pose. Additionally it can 
  make use of the information from /velocity topic. 

  It also needs the reference trajectory published as a 
  geometry_msgs/Pose2D message under topic /reference_pose. 
  It can also use the /reference_velocity information.
  As a result it publishes control signals under topic 
  /controls.

  Mateusz Przybyla
  Chair of Control and Systems Engineering
  Faculty of Computing
  Poznan University of Technology
***/


class AutomaticController {
public:
  float x, y, theta;        // Actual pose
  float v, w;               // Actual velocity
  float x_r, y_r, theta_r;  // Reference pose
  float v_r, w_r;           // Reference velocity  
  float t;                  // Time

  geometry_msgs::Twist controls;

  ros::Publisher  ctrl_pub;
  ros::Subscriber pos_sub;
  ros::Subscriber vel_sub;
  ros::Subscriber ref_pos_sub;
  ros::Subscriber ref_vel_sub;

  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pos_msg)
  {
    this->x = pos_msg->x;
    this->y = pos_msg->y;
    this->theta = pos_msg->theta;
  }

  void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel_msg)
  {
    this->v = vel_msg->linear.x;
    this->w = vel_msg->angular.z;
  }

  void refPoseCallback(const geometry_msgs::Pose2D::ConstPtr& ref_pos_msg)
  {
    this->x_r = ref_pos_msg->x;
    this->y_r = ref_pos_msg->y;
    this->theta_r = ref_pos_msg->theta;
  }

  void refVelocityCallback(const geometry_msgs::Twist::ConstPtr& ref_vel_msg)
  {
    this->v_r = ref_vel_msg->linear.x;
    this->w_r = ref_vel_msg->angular.z;
  }

  void computeControls()
  { 
    // HERE PUT THE CODE
  
    controls.linear.x = 1.0;
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
