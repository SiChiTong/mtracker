#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

/***
  This node simulates the behavior of a real MTracker
  robot. It receives the /controls signals and publish
  the virtual odometry information as /virtual_pose and 
  /virtual_velocity. It also broadcasts the tf frame 
  /virtual_base. The node works in synchronous manner
  with the rate of 100 Hz.

  Mateusz Przybyla
  Chair of Control and Systems Engineering
  Faculty of Computing
  Poznan University of Technology
***/


class Simulator
{
public:
  Simulator() : dt(0.01f), Tf(0.02f) {}

  float dt;  // Sampling time
  float Tf;  // Filtering time constant

  ros::Subscriber ctrl_sub;
  ros::Publisher  pose_pub;
  ros::Publisher  vel_pub;

  geometry_msgs::Pose2D pose;      // Pose of the virtual robot
  geometry_msgs::Twist  velocity;  // Velocity of the virtual robot
  geometry_msgs::Twist  controls;

  void ctrlCallback(const geometry_msgs::Twist::ConstPtr& controls_msg)
  {
    controls.linear.x  = controls_msg->linear.x;
    controls.angular.z = controls_msg->angular.z;
  }

  void computeVelocity()
  {
    velocity.linear.x  += dt / (dt + Tf) * (controls.linear.x - velocity.linear.x);
    velocity.angular.z += dt / (dt + Tf) * (controls.angular.z - velocity.angular.z);
  }

  void computePose()
  {
    pose.x += dt * velocity.linear.x * cos(pose.theta);
    pose.y += dt * velocity.linear.x * sin(pose.theta);
    pose.theta += dt * velocity.angular.z;
  }

  void publishPose()
  {
    tf::TransformBroadcaster pose_bc;
    geometry_msgs::TransformStamped pose_tf;

    pose_tf.header.stamp = ros::Time::now();
    pose_tf.header.frame_id = "/virtual_base";
    pose_tf.transform.translation.x = pose.x;
    pose_tf.transform.translation.y = pose.y;
    pose_tf.transform.translation.z = 0.0;
    pose_tf.transform.rotation = tf::createQuaternionMsgFromYaw(pose.theta);

    pose_bc.sendTransform(pose_tf);
    pose_pub.publish(pose);
  }

  void publishVelocity()
  {
    vel_pub.publish(velocity);
  }
};


int main(int argc, char **argv)
{
  Simulator s;

  ros::init(argc, argv, "mtracker_simulator");	//, ros::init_options::NoSigintHandler
  ros::NodeHandle n;

  s.pose_pub = n.advertise<geometry_msgs::Pose2D>("/virtual_pose", 10);
  s.vel_pub  = n.advertise<geometry_msgs::Twist>("/virtual_velocity", 10);
  s.ctrl_sub = n.subscribe("/controls", 10, &Simulator::ctrlCallback, &s);

  ROS_INFO("MTracker simulator start");

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();

    s.computeVelocity();
    s.publishVelocity();
    s.computePose();
    s.publishPose();

    rate.sleep();
  }

  return 0;
}
