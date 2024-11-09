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

#ifndef DWB_CORE__PUBLISHER_HPP_
#define DWB_CORE__PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <dwb_core/trajectory_critic.hpp>
#include <dwb_msgs/LocalPlanEvaluation.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>


namespace dwb_core
{

/**
 * @class DWBPublisher
 * @brief Consolidation of all the publishing logic for the DWB Local Planner.
 *
 * Right now, it can publish
 *   1) The Global Plan (as passed in using setPath)
 *   2) The Local Plan (after it is calculated)
 *   3) The Transformed Global Plan (since it may be different than the global)
 *   4) The Full LocalPlanEvaluation
 *   5) Markers representing the different trajectories evaluated
 *   6) The CostGrid (in the form of a complex PointCloud2)
 */
class DWBPublisher
{
public:
  DWBPublisher(ros::NodeHandle & parent, const std::string & plugin_name);

  void initialize(ros::NodeHandle &nh);

  /**
   * @brief Does the publisher require that the LocalPlanEvaluation be saved
   * @return True if the Evaluation is needed to publish either directly or as trajectories
   */
  bool shouldRecordEvaluation() {return publish_evaluation_ || publish_trajectories_;}

  /**
   * @brief If the pointer is not null, publish the evaluation and trajectories as needed
   */
  void publishEvaluation(std::shared_ptr<dwb_msgs::LocalPlanEvaluation> results);
  void publishLocalPlan(
    const std_msgs::Header & header,
    const dwb_msgs::Trajectory2D & traj);
  void publishCostGrid(
    const costmap_2d::Costmap2DROS* costmap_ros,
    const std::vector<TrajectoryCritic::Ptr> critics);
  void publishGlobalPlan(const nav_2d_msgs::Path2D plan);
  void publishTransformedPlan(const nav_2d_msgs::Path2D plan);
  void publishLocalPlan(const nav_2d_msgs::Path2D plan);

protected:
  void publishTrajectories(const dwb_msgs::LocalPlanEvaluation & results);

  // Helper function for publishing other plans
  void publishGenericPlan(
    const nav_2d_msgs::Path2D plan,
    ros::Publisher & pub, bool flag);

  // Flags for turning on/off publishing specific components
  bool publish_evaluation_;
  bool publish_global_plan_;
  bool publish_transformed_;
  bool publish_local_plan_;
  bool publish_trajectories_;
  bool publish_cost_grid_pc_;
  bool publish_input_params_;
  bool initialized_;

  // Publisher Objects
  ros::Publisher eval_pub_;
  ros::Publisher global_pub_;
  ros::Publisher transformed_pub_;
  ros::Publisher local_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher cost_grid_pc_pub_;
};

}  // namespace dwb_core

#endif  // DWB_CORE__PUBLISHER_HPP_
