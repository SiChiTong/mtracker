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
  ros::Publisher ref_pos_pub;
  ros::Publisher ref_vel_pub;

  geometry_msgs::Pose2D ref_pose;
  geometry_msgs::Twist  ref_velocity;

  void calculateRefPose(float t)
  {
    static float omega = 0.3f;
    static float R = 0.7f;

    ref_pose.x = R * cos(omega * t);
    ref_pose.y = R * sin(omega * t);
    ref_pose.theta = omega * t;
  }

  void calculateRefVelocity(float t)
  {
    static float omega = 0.3f;
    static float R = 0.7f;

    ref_velocity.linear.x  = R * omega;
    ref_velocity.angular.z = omega;
  }

  void publishRefPose()
  {
    static tf::TransformBroadcaster pose_bc;
    static tf::Transform pose_tf;

    pose_tf.setOrigin(tf::Vector3(ref_pose.x, ref_pose.y, 0.0));
    pose_tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, ref_pose.theta));
    pose_bc.sendTransform(tf::StampedTransform(pose_tf, ros::Time::now(), "/world", "/reference"));

    ref_pos_pub.publish(ref_pose);
  }

  void publishRefVelocity()
  {
    ref_vel_pub.publish(ref_velocity);
  }
};


int main(int argc, char **argv)
{
  ReferenceGenerator r;

  ros::init(argc, argv, "reference_generator");
  ros::NodeHandle n;

  r.ref_pos_pub = n.advertise<geometry_msgs::Pose2D>("/reference_pose", 10);
  r.ref_vel_pub = n.advertise<geometry_msgs::Twist>("/reference_velocity", 10);

  ROS_INFO("MTracker reference generator start");

  ros::Rate rate(100.0);
  ros::Time tic = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();

    ros::Time toc = ros::Time::now();
    float t = (toc - tic).toSec();

    r.calculateRefPose(t);
    r.publishRefPose();
    r.calculateRefVelocity(t);
    r.publishRefVelocity();

    rate.sleep();
  }

  return 0;
}
