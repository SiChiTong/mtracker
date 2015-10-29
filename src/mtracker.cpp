/***
 * This node interconnects the high-level controller
 * of the MTracker robot with the low-level driver
 * implemented on the robots microprocessor. It
 * subscribes to messeges of type geometry_msgs/Twist
 * published under /controls topic and publish the
 * odometry information obtained from the robot as
 * geometry_msgs/Pose2D message under topic /pose
 * as well as geometry_msgs/Twist message under topic
 * velocity.

 * This node works in synchronous manner, meaning that
 * it has quasi-constant cycle time set to 100 Hz.

 * Mateusz Przybyla
 * Chair of Control and Systems Engineering
 * Faculty of Computing
 * Poznan University of Technology
***/

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <signal.h>
#include <memory>

#include "serial.hpp"

namespace mtracker
{

class Robot
{
public:
  Robot() : nh_(""), nh_params_("~"), com_(new Serial),
    ROBOT_BASE(0.145f), WHEEL_RADIUS(0.025f), loop_rate_(100)
  {
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("pose", 10);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("velocity", 10);
    controls_sub_ = nh_.subscribe("controls", 10, &Robot::controlsCallback, this);
    trigger_srv_ = nh_.advertiseService("trigger_motors", &Robot::triggerCallback, this);

    if (com_->openPort())
    {
      switchMotors(true);
      setWheelsVelocities(0.0f, 0.0f);
      setOdometryPose(0.0f, 0.0f, 0.0f);
      ROS_INFO("MTracker start");
    }
    else
    {
      ROS_INFO("Could not open COM port.");
      ros::shutdown();
    }
  }

  ~Robot()
  {
    setWheelsVelocities(0.0f, 0.0f);
    switchMotors(false);
    delete com_;
  }

  void publishPose()
  {
    pose_pub_.publish(pose_);
  }

  void publishVelocity()
  {
    velocity_pub_.publish(velocity_);
  }

  void publishTransform()
  {
    pose_tf_.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
    pose_tf_.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, pose_.theta));
    pose_bc_.sendTransform(tf::StampedTransform(pose_tf_, ros::Time::now(), "world", "robot"));
  }

  bool readData() { return com_->readFrame(); }
  int getLoopRate() const { return loop_rate_; }

private:
  const float ROBOT_BASE;
  const float WHEEL_RADIUS;

  void controlsCallback(const geometry_msgs::Twist controls_msg)
  {
    float w_l = (controls_msg.linear.x - ROBOT_BASE * controls_msg.angular.z / 2.0) / WHEEL_RADIUS;
    float w_r = (controls_msg.linear.x + ROBOT_BASE * controls_msg.angular.z / 2.0) / WHEEL_RADIUS;

    setWheelsVelocities(w_l, w_r);
  }

  bool triggerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    switchMotors(true);
    return true;
  }

  int loop_rate_;

  Serial* com_;

  geometry_msgs::Pose2D pose_;
  geometry_msgs::Twist  velocity_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_params_;

  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::Subscriber controls_sub_;
  ros::ServiceServer trigger_srv_;

  tf::TransformBroadcaster pose_bc_;
  tf::Transform pose_tf_;

  void switchMotors(bool motors_on)
  {
    if (motors_on)
      com_->setMode(MODE_MOTORS_ON);
    else
      com_->setMode(MODE_MOTORS_OFF);

    com_->writeFrame();
  }

  void setWheelsVelocities(float w_l, float w_r)
  {
    com_->setVelocities(w_l, w_r);
    com_->setMode(MODE_MOTORS_ON);
    com_->writeFrame();
  }

  void setOdometryPose(float x, float y, float theta)
  {
    com_->setPose(x, y, theta);
    com_->setMode(MODE_SET_ODOMETRY);
    com_->writeFrame();
  }

  geometry_msgs::Pose2D getRobotPose()
  {
    return com_->getPose();
  }

  geometry_msgs::Twist getRobotVelocity()
  {
    geometry_msgs::Twist velocity;

    float w_l = com_->getVelocities().angular.x;
    float w_r = com_->getVelocities().angular.y;

    velocity.linear.x  = (w_r + w_l) * WHEEL_RADIUS / 2.0;
    velocity.angular.z = (w_r - w_l) * WHEEL_RADIUS / ROBOT_BASE;

    return velocity;
  }
};

} // end namespace mtracker


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtracker");
  mtracker::Robot r;

  ros::Rate rate(r.getLoopRate());
  while (ros::ok())
  {
    ros::spinOnce();

    if (r.readData())
    {
      r.publishPose();
      r.publishVelocity();
      r.publishTransform();
    }

    rate.sleep();
  }

  return 0;
}
