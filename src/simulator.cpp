#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"


class Simulator
{
public:
  ros::Subscriber ctrl_sub;
  ros::Publisher  pose_pub;

  geometry_msgs::Pose2D pose;
  geometry_msgs::Twist  controls;

  void ctrlCallback(const geometry_msgs::Twist::ConstPtr& controls_msg)
  {
    controls.linear.x  = controls_msg->linear.x;
    controls.angular.z = controls_msg->angular.z;
  }

  void computePose()
  {
    pose.x += 0.01 * controls.linear.x * cos(pose.theta);
    pose.y += 0.01 * controls.linear.x * sin(pose.theta);
    pose.theta += 0.01 * controls.angular.z;
  }

  void publishPose()
  {
    tf::TransformBroadcaster pose_bc;
    geometry_msgs::TransformStamped pose_tf;

    pose_tf.header.stamp = ros::Time::now();
    pose_tf.header.frame_id = "/base";
    pose_tf.transform.translation.x = pose.x;
    pose_tf.transform.translation.y = pose.y;
    pose_tf.transform.translation.z = 0.0;
    pose_tf.transform.rotation = tf::createQuaternionMsgFromYaw(pose.theta);

    pose_bc.sendTransform(pose_tf);
    pose_pub.publish(pose);
  }
};


int main(int argc, char **argv)
{
  Simulator s;

  ros::init(argc, argv, "mtracker_simulator");
  ros::NodeHandle n;

  s.pose_pub = n.advertise<geometry_msgs::Pose2D>("/pose", 10);
  s.ctrl_sub = n.subscribe("/controls", 10, &Simulator::ctrlCallback, &s);

  ROS_INFO("MTracker Simulator");

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();

    s.computePose();
    s.publishPose();

    rate.sleep();
  }

  return 0;
}
