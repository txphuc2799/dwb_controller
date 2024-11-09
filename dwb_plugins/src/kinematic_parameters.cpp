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

#include <dwb_plugins/kinematic_parameters.hpp>

#include <memory>
#include <string>
#include <vector>

using std::placeholders::_1;

namespace dwb_plugins
{

KinematicsHandler::KinematicsHandler():
  initialized_(false)
{
  kinematics_.store(new KinematicParameters);
}

KinematicsHandler::~KinematicsHandler()
{
  delete kinematics_.load();
  if (dynamic_srv_) {
    delete dynamic_srv_;
    dynamic_srv_ = nullptr;  // Optional: Helps to avoid dangling pointer issues.
  }
}

void KinematicsHandler::initialize(
  const ros::NodeHandle & nh,
  const std::string & plugin_name)
{
  if (!initialized_) {
    nh_ = nh;
    plugin_name_ = plugin_name;

    KinematicParameters kinematics;

    nh_.param("min_vel_x", kinematics.min_vel_x_, 0.0);
    nh_.param("min_vel_y", kinematics.min_vel_y_, 0.0);
    nh_.param("max_vel_x", kinematics.max_vel_x_, 0.7);
    nh_.param("max_vel_y", kinematics.max_vel_y_, 0.0);
    nh_.param("max_vel_theta", kinematics.max_vel_theta_, 0.5);
    nh_.param("min_speed_xy", kinematics.min_speed_xy_, 0.0);
    nh_.param("max_speed_xy", kinematics.max_speed_xy_, 0.7);
    nh_.param("min_speed_theta", kinematics.min_speed_theta_, 0.0);
    nh_.param("acc_lim_x", kinematics.acc_lim_x_, 1.0);
    nh_.param("acc_lim_y", kinematics.acc_lim_y_, 0.0);
    nh_.param("acc_lim_theta", kinematics.acc_lim_theta_, 2.0);
    nh_.param("decel_lim_x", kinematics.decel_lim_x_, -2.5);
    nh_.param("decel_lim_y", kinematics.decel_lim_y_, 0.0);
    nh_.param("decel_lim_theta", kinematics.decel_lim_theta_, -3.2);

    kinematics.base_max_vel_x_ = kinematics.max_vel_x_;
    kinematics.base_max_vel_y_ = kinematics.max_vel_y_;
    kinematics.base_max_speed_xy_ = kinematics.max_speed_xy_;
    kinematics.base_max_vel_theta_ = kinematics.max_vel_theta_;

    kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
    kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

    update_kinematics(kinematics);

    // Set up parameter reconfigure
    dynamic_srv_ = new ParamterConfigServer(nh_);
    CallbackType cb = boost::bind(&KinematicsHandler::reconfigureCB, this,
                                  boost::placeholders::_1, boost::placeholders::_2);
    dynamic_srv_->setCallback(cb);
    initialized_ = true;
  }
}

void KinematicsHandler::reconfigureCB(Config& config, uint32_t level)
{
  if (!initialized_) {
    return;
  }
  KinematicParameters kinematics(*kinematics_.load());
  ROS_INFO("DWBController: ""Got e new reconfigure.");
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  kinematics.min_vel_x_ = config.min_vel_x;
  kinematics.min_vel_y_ = config.min_vel_y;
  kinematics.max_vel_x_ = config.max_vel_x;
  kinematics.max_vel_y_ = config.max_vel_y;
  kinematics.max_vel_theta_ = config.max_vel_theta;

  kinematics.min_speed_xy_ = config.min_speed_xy;
  kinematics.max_speed_xy_ = config.max_speed_xy;
  kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;
  kinematics.min_speed_theta_ = config.min_speed_theta;

  kinematics.acc_lim_x_ = config.acc_lim_x;
  kinematics.acc_lim_y_ = config.acc_lim_y;
  kinematics.acc_lim_theta_ = config.acc_lim_theta;
  kinematics.decel_lim_x_ = config.decel_lim_x;
  kinematics.decel_lim_y_ = config.decel_lim_y;
  kinematics.decel_lim_theta_ = config.decel_lim_theta;
  update_kinematics(kinematics);
}

void KinematicsHandler::update_kinematics(KinematicParameters kinematics)
{
  delete kinematics_.load();
  kinematics_.store(new KinematicParameters(kinematics));
}

}  // namespace dwb_plugins
