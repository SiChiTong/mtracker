#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

/***
 * This node simulates the behavior of a real MTracker
 * robot. It receives the /controls signals and publish
 * the virtual odometry information as /virtual_pose and
 * /virtual_velocity. It also broadcasts the tf frame
 * /virtual_base. The node works in synchronous manner
 * with the default rate of 100 Hz.

 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/


struct Simulator
{
  geometry_msgs::Pose2D pose;
  geometry_msgs::Twist  velocity;
  geometry_msgs::Twist  controls;

  double Tp, Tf;
  bool  publish_tf;

  void controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg)
  {
    controls = *controls_msg;
  }

  void computeVelocity()
  {
    velocity.linear.x  += Tp / (Tp + Tf) * (controls.linear.x - velocity.linear.x);
    velocity.angular.z += Tp / (Tp + Tf) * (controls.angular.z - velocity.angular.z);
  }

  void computePose()
  {
    pose.x += Tp * velocity.linear.x * cos(pose.theta);
    pose.y += Tp * velocity.linear.x * sin(pose.theta);
    pose.theta += Tp * velocity.angular.z;
  }

  void updateParams(const ros::TimerEvent& event)
  {
    // Global params
    static ros::NodeHandle nh;

    if (!nh.getParam("sampling_time", Tp))
    {
      Tp = 0.01;
      nh.setParam("sampling_time", Tp);
    }

    // Local params
    static ros::NodeHandle nh_priv("~");

    if (!nh_priv.getParam("robot_inertia", Tf))
      Tf = 0.1;

    if (!nh_priv.getParam("publish_tf", publish_tf))
      publish_tf = true;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtracker_simulator");
  ros::NodeHandle n;

  Simulator s;

  {
    ros::TimerEvent e;
    s.updateParams(e);
  }

  ros::Subscriber controls_sub = n.subscribe("/controls", 10, &Simulator::controlsCallback, &s);
  ros::Publisher  pose_pub     = n.advertise<geometry_msgs::Pose2D>("/virtual_pose", 10);
  ros::Publisher  velocity_pub = n.advertise<geometry_msgs::Twist>("/virtual_velocity", 10);
  ros::Timer      params_tim   = n.createTimer(ros::Duration(0.25), &Simulator::updateParams, &s);

  ROS_INFO("MTracker simulator start");

  ros::Rate rate(1.0 / s.Tp);

  while (ros::ok())
  {
    ros::spinOnce();

    s.computeVelocity();
    s.computePose();

    if (s.publish_tf)
    {
      static tf::TransformBroadcaster pose_bc;
      static tf::Transform pose_tf;

      pose_tf.setOrigin(tf::Vector3(s.pose.x, s.pose.y, 0.0));
      pose_tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, s.pose.theta));
      pose_bc.sendTransform(tf::StampedTransform(pose_tf, ros::Time::now(), "/world", "/virtual_robot"));
    }

    velocity_pub.publish(s.velocity);
    pose_pub.publish(s.pose);

    rate.sleep();
  }

  return 0;
}
