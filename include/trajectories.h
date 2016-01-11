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

#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <cmath>
#include <geometry_msgs/Vector3.h>

namespace mtracker
{

class Trajectory
{
public:
  Trajectory() : x_0_(0.0), y_0_(0.0), phi_0_(0.0) {}
  Trajectory(double x, double y, double phi) : x_0_(x), y_0_(y), phi_0_(phi) {}

  virtual geometry_msgs::Vector3 calculatePose(double t) {
    geometry_msgs::Vector3 pose;
    pose.x = x_0_;
    pose.y = y_0_;
    pose.z = phi_0_;

    return pose;
  }

  virtual geometry_msgs::Vector3 calculateVelocity(double t) {
    geometry_msgs::Vector3 velocity;
    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.0;

    return velocity;
  }

  void setOrigin(double x, double y, double phi) {
    x_0_ = x;
    y_0_ = y;
    phi_0_ = phi;
  }

protected:
  double x_0_;
  double y_0_;
  double phi_0_;
};


class LinearTrajectory : public Trajectory
{
public:
  LinearTrajectory() : v_(0.0), phi_(0.0) {}
  LinearTrajectory(double v, double phi) : v_(v), phi_(phi) {}
  LinearTrajectory(double x, double y, double v, double phi) : Trajectory(x, y, phi), v_(v), phi_(phi) {}

  virtual geometry_msgs::Vector3 calculatePose(double t) {
    geometry_msgs::Vector3 pose;
    pose.x = x_0_ + v_ * cos(phi_) * t;
    pose.y = y_0_ + v_ * sin(phi_) * t;
    pose.z = phi_;

    return pose;
  }

  virtual geometry_msgs::Vector3 calculateVelocity(double t) {
    geometry_msgs::Vector3 velocity;
    velocity.x = v_ * cos(phi_);
    velocity.y = v_ * sin(phi_);
    velocity.z = 0.0;

    return velocity;
  }

protected:
  double v_;         // Velocity [m/s]
  double phi_;       // Orientation [rad]
};


class HarmonicTrajectory : public Trajectory
{
public:
  HarmonicTrajectory() : w_(0.0), r_x_(0.0), r_y_(0.0), n_x_(0.0), n_y_(0.0) {}
  HarmonicTrajectory(double T, double r_x, double r_y, int n_x, int n_y) :
    Trajectory(0.0, 0.0, M_PI_2), r_x_(r_x), r_y_(r_y), n_x_(n_x), n_y_(n_y)
    { (T != 0) ? w_ = 2 * M_PI / T : w_ = 0.0; }
  HarmonicTrajectory(double x, double y, double T, double r_x, double r_y, int n_x, int n_y) :
    Trajectory(x, y, M_PI_2), r_x_(r_x), r_y_(r_y), n_x_(n_x), n_y_(n_y)
    { (T != 0) ? w_ = 2 * M_PI / T : w_ = 0.0; }

  virtual geometry_msgs::Vector3 calculatePose(double t) {
    geometry_msgs::Vector3 pose;
    pose.x = x_0_ + r_x_ * cos(n_x_ * w_ * t);
    pose.y = y_0_ + r_y_ * sin(n_y_ * w_ * t);
    pose.z = atan2(r_y_ * n_y_ * w_ * cos(n_y_ * w_ * t), -r_x_ * n_x_ * w_ * sin(n_x_ * w_ * t));

    return pose;
  }

  virtual geometry_msgs::Vector3 calculateVelocity(double t) {
    geometry_msgs::Vector3 velocity;
    velocity.x = -r_x_ * n_x_ * w_ * sin(n_x_ * w_ * t);
    velocity.y =  r_y_ * n_y_ * w_ * cos(n_y_ * w_ * t);
    velocity.z = (-r_y_ * pow(n_y_, 2.0) * pow(w_, 2.0) * sin(n_y_ * w_ * t) * velocity.x -
                  -r_x_ * pow(n_x_, 2.0) * pow(w_, 2.0) * cos(n_x_ * w_ * t) * velocity.y) /
                 (pow(velocity.x, 2.0) + pow(velocity.y, 2.0));

    return velocity;
  }

protected:
  double w_;           // Frequency [rad/s]
  int n_x_, n_y_;      // Frequency multipliers [-]
  double r_x_, r_y_;   // Radii [m]
};


class LemniscateTrajectory : public Trajectory
{
public:
  LemniscateTrajectory() : w_(0.0), a_(0.0), b_(0.0) {}
  LemniscateTrajectory(double T, double a, double b) : a_(a), b_(b)
    { (T != 0) ? w_ = 2 * M_PI / T : w_ = 0.0; }
  LemniscateTrajectory(double x, double y, double T, double a, double b) : Trajectory(x, y, M_PI_2), a_(a), b_(b)
    { (T != 0) ? w_ = 2 * M_PI / T : w_ = 0.0; }

  virtual geometry_msgs::Vector3 calculatePose(double t) {
    geometry_msgs::Vector3 pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.z = 0.0;

    return pose;
  }

  virtual geometry_msgs::Vector3 calculateVelocity(double t) {
    geometry_msgs::Vector3 velocity;
    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.0;

    return velocity;
  }

protected:
  double w_;      // Frequency [rad/s]
  double a_;      // Separation of focal [m]
  double b_;      // Distance (product) [m]
};

} // namespace mtracker

#endif // TRAJECTORIES_H
