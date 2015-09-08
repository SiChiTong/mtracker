#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

/***
 * This is a time-based reference trajectory generator
 * used for the automatic controller of the MTracker.
 * It calculates the pose and velocity of a reference
 * robot and publishes them under topics reference_pose
 * and reference_velocity, respectively. It also
 * broadcasts the tf frame called reference_robot. The
 * node works synchronously with a rate of 100 Hz.
 *
 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/


struct ReferenceGenerator
{
  geometry_msgs::Pose2D ref_pose;
  geometry_msgs::Twist  ref_velocity;

  // Parameters
  enum TrajectoryType { POINT, LINEAR, CIRCULAR, POLYNOMIAL } trajectory;
  double x_o, y_o, theta_o;
  double v, w;
  double R;
  int loop_rate;
  bool publish_tf;

  void calculateRefPose(double t)
  {
    switch (trajectory)
    {
    case POINT:
      ref_pose.x = x_o;
      ref_pose.y = y_o;
      ref_pose.theta = theta_o;
      break;
    case LINEAR:
      ref_pose.x = x_o + v * cos(theta_o) * t;
      ref_pose.y = y_o + v * sin(theta_o) * t;
      ref_pose.theta = theta_o;
      break;
    case CIRCULAR:
      ref_pose.x = x_o + R * cos(w * t);
      ref_pose.y = y_o + R * sin(w * t);
      ref_pose.theta = w * t + M_PI_2;
      break;
    }
  }

  void calculateRefVelocity(double t)
  {
    switch (trajectory)
    {
    case POINT:
      ref_velocity.linear.x  = 0.0;
      ref_velocity.linear.y  = 0.0;
      ref_velocity.angular.z = 0.0;
      break;
    case LINEAR:
      ref_velocity.linear.x  = v * cos(theta_o);
      ref_velocity.linear.y  = v * sin(theta_o);
      ref_velocity.angular.z = 0.0;
      break;
    case CIRCULAR:
      ref_velocity.linear.x  = -R * w * sin(w * t);
      ref_velocity.linear.y  =  R * w * cos(w * t);
      ref_velocity.angular.z =  w;
      break;
    }
  }

  void updateParams(const ros::TimerEvent& event)
  {
    static ros::NodeHandle nh_global;
    static ros::NodeHandle nh_local("~");

    if (!nh_global.getParam("loop_rate", loop_rate))
    {
      loop_rate = 100;
      nh_global.setParam("loop_rate", loop_rate);
    }

    int trajectory_type;
    if (!nh_local.getParam("trajectory_type", trajectory_type))
      trajectory_type == 0;

    if (trajectory_type == 1) trajectory = LINEAR;
    else if (trajectory_type == 2) trajectory = CIRCULAR;
    else trajectory = POINT;

    if (!nh_local.getParam("x_origin", x_o))
      x_o = 0.0;
    if (!nh_local.getParam("y_origin", y_o))
      y_o = 0.0;
    if (!nh_local.getParam("theta_origin", theta_o))
      theta_o = 0.0;
    if (!nh_local.getParam("velocity", v))
      v = 0.0;
    if (!nh_local.getParam("omega", w))
      w = 0.0;
    if (!nh_local.getParam("radius", R))
      R = 0.0;
    if (!nh_local.getParam("publish_tf", publish_tf))
      publish_tf = true;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "reference_generator");
  ros::NodeHandle n;

  ReferenceGenerator r;

  {
    ros::TimerEvent e;
    r.updateParams(e);
  }

  ros::Publisher ref_pose_pub     = n.advertise<geometry_msgs::Pose2D>("reference_pose", 10);
  ros::Publisher ref_velocity_pub = n.advertise<geometry_msgs::Twist>("reference_velocity", 10);
  ros::Timer     params_tim       = n.createTimer(ros::Duration(0.25), &ReferenceGenerator::updateParams, &r);

  ROS_INFO("MTracker reference generator start");

  ros::Rate rate(r.loop_rate);
  ros::Time tic = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();

    ros::Time toc = ros::Time::now();
    float t = (toc - tic).toSec();

    r.calculateRefPose(t);
    r.calculateRefVelocity(t);

    if (r.publish_tf)
    {
      static tf::TransformBroadcaster ref_pose_bc;
      static tf::Transform ref_pose_tf;

      ref_pose_tf.setOrigin(tf::Vector3(r.ref_pose.x, r.ref_pose.y, 0.0));
      ref_pose_tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, r.ref_pose.theta));
      ref_pose_bc.sendTransform(tf::StampedTransform(ref_pose_tf, ros::Time::now(), "world", "reference_robot"));
    }

    ref_pose_pub.publish(r.ref_pose);
    ref_velocity_pub.publish(r.ref_velocity);

    rate.sleep();
  }

  return 0;
}
