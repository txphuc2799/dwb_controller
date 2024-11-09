/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <dwb_critics/oscillation.hpp>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <dwb_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dwb_critics::OscillationCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void OscillationCritic::onInit()
{
  nh_.param("oscillation_distance", oscillation_distance_, 0.1);
  nh_.param("oscillation_time_threshold", oscillation_time_threshold_, 5.0);
  reset();
}

bool OscillationCritic::prepare(
  const geometry_msgs::Pose2D & robot_pose,
  const nav_2d_msgs::Pose2DStamped & goal_pose,
  const nav_2d_msgs::Twist2D &,
  const geometry_msgs::Pose2D & transformed_end_pose,
  const nav_2d_msgs::Path2D &)
{
  if (prev_goal_pose_.pose != goal_pose.pose) {
    has_stopped_ = true;
    prev_goal_pose_.pose = goal_pose.pose;
  }
  double dx = robot_pose.x - transformed_end_pose.x,
         dy = robot_pose.y - transformed_end_pose.y;
  
  if (has_stopped_) {
    if (dx*dx + dy*dy <= oscillation_distance_) {
      allow_oscillation_ = true;
      has_stopped_ = false;
    } else {
      allow_oscillation_ = false;
    }
  }
  return true;
}

void OscillationCritic::debrief(nav_2d_msgs::Twist2D & cmd_vel, bool &oscillation_state)
{
  if (allow_oscillation_) {
    if (cmd_vel.x == 0.0) {
      if (has_x_stopped_) {
        prev_reset_time_ = ros::Time::now();
        has_x_stopped_ = false;
      }
      double time_diff = (ros::Time::now() - prev_reset_time_).toSec();
      if (time_diff > oscillation_time_threshold_) {
        oscillation_state = true;
        ROS_WARN("DWBController: Robot is oscillating!!!");
      }
    } else {
      has_x_stopped_ = true;
    }
  } else {
    oscillation_state = false;
  }
}

void OscillationCritic::reset()
{
}

double OscillationCritic::scoreTrajectory(const dwb_msgs::Trajectory2D & traj)
{
  return 0.0;
}

}  // namespace dwb_critics
