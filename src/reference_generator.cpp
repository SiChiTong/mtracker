#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

/***
  This is a time-based reference trajectory generator 
  used for the automatic controller of the MTracker. 
  It calculates the pose and velocity of a reference 
  robot and publishes them under topics /reference_pose
  and /reference_velocity, respectively. It also 
  broadcasts the tf frame called /reference. The node
  works synchronously with a rate of 100 Hz.

  Mateusz Przybyla
  Chair of Control and Systems Engineering
  Faculty of Computing
  Poznan University of Technology
***/


class ReferenceGenerator
{
public:
  ReferenceGenerator() : x_r(0.0f), y_r(0.0f), theta_r(0.0f), v_r(0.0f), w_r(0.0f), t(0.0f) {}

  float x_r, y_r, theta_r;
  float v_r, w_r;
  float t;

  geometry_msgs::Pose2D reference_pose;
  geometry_msgs::Twist  reference_velocity;

  ros::Publisher ref_pos_pub;
  ros::Publisher ref_vel_pub;

  void calculateRefPose()
  {
    float omega = 1.0;
    x_r = 1.0 * cos(omega * t);
    y_r = 1.0 * sin(omega * t);
    theta_r = omega * t;
  }

  void calculateRefVelocity()
  {
    float omega = 1.0f;
    v_r = 1.0 * omega;
    w_r = omega;
  }

  void publishRefPose()
  {
    reference_pose.x = x_r;
    reference_pose.y = y_r;
    reference_pose.theta = theta_r;

    ref_pos_pub.publish(reference_pose);
  }

  void publishRefVelocity()
  {
    
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reference_generator");

  ROS_INFO("MTracker reference generator");

  return 0;
}
