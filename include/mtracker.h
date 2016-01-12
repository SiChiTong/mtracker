/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#ifndef MTRACKER_H
#define MTRACKER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <signal.h>
#include <memory>

#include "serial.h"

namespace mtracker
{

class MTracker
{
public:
  MTracker();

private:
  const double ROBOT_BASE;
  const double WHEEL_RADIUS;

  void controlsCallback(const geometry_msgs::Twist::ConstPtr& controls_msg);

  void initialize();
  void sendData();
  void readData();

  void switchMotors(bool motors_on);
  void setVelocities();
  void stopWheels();

  void publishTransform();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber controls_sub_;
  ros::Publisher odom_pose_pub_;
  ros::Publisher odom_velocity_pub_;
  ros::Publisher path_pub_;

  tf::TransformBroadcaster pose_bc_;
  tf::Transform pose_tf_;

  geometry_msgs::Pose2D odom_pose_;
  geometry_msgs::Twist odom_velocity_;
  geometry_msgs::Twist controls_;

  Serial* com_;

  int loop_rate_;
};

}

#endif // MTRACKER_H
