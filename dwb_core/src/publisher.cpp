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

#include <dwb_core/publisher.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_2d_utils/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using std::max;
using std::string;

namespace dwb_core
{

DWBPublisher::DWBPublisher(ros::NodeHandle & parent, const std::string & plugin_name)
{
  initialize(parent);
}

void DWBPublisher::initialize(ros::NodeHandle &nh)
{
  if (!initialized_) {
    nh.param("publish_evaluation", publish_evaluation_, true);
    nh.param("publish_global_plan", publish_global_plan_, true);
    nh.param("publish_transformed_plan", publish_transformed_, true);
    nh.param("publish_local_plan", publish_local_plan_, true);
    nh.param("publish_trajectories", publish_trajectories_, true);
    nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    nh.param("marker_lifetime", 0.1);

    eval_pub_ = nh.advertise<dwb_msgs::LocalPlanEvaluation>("evaluation", 1);
    global_pub_ = nh.advertise<nav_msgs::Path>("received_global_plan", 1);
    transformed_pub_ = nh.advertise<nav_msgs::Path>("transformed_global_plan", 1);
    local_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
    cost_grid_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cost_cloud", 1);

    initialized_ = true;
  }
}

void DWBPublisher::publishEvaluation(std::shared_ptr<dwb_msgs::LocalPlanEvaluation> results)
{
  if (results) {
    if (publish_evaluation_ && eval_pub_.getNumSubscribers() > 0) {
      // auto msg = std::make_unique<dwb_msgs::LocalPlanEvaluation>(*results);
      eval_pub_.publish(*results);
    }
    publishTrajectories(*results);
  }
}

void DWBPublisher::publishTrajectories(const dwb_msgs::LocalPlanEvaluation & results)
{
  if (marker_pub_.getNumSubscribers() < 1) {return;}

  if (!publish_trajectories_) {return;}
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker m;

  if (results.twists.size() == 0) {return;}

  geometry_msgs::Point pt;

  m.header = results.header;
  m.type = m.LINE_STRIP;
  m.pose.orientation.w = 1;
  m.scale.x = 0.002;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0.1);

  double best_cost = results.twists[results.best_index].total;
  double worst_cost = results.twists[results.worst_index].total;
  double denominator = worst_cost - best_cost;

  if (std::fabs(denominator) < 1e-9) {
    denominator = 1.0;
  }

  unsigned currentValidId = 0;
  unsigned currentInvalidId = 0;
  string validNamespace("ValidTrajectories");
  string invalidNamespace("InvalidTrajectories");
  for (unsigned int i = 0; i < results.twists.size(); i++) {
    const dwb_msgs::TrajectoryScore & twist = results.twists[i];
    double displayLevel = (twist.total - best_cost) / denominator;
    if (twist.total >= 0) {
      m.color.r = displayLevel;
      m.color.g = 1.0 - displayLevel;
      m.color.b = 0;
      m.color.a = 1.0;
      m.ns = validNamespace;
      m.id = currentValidId;
      ++currentValidId;
    } else {
      m.color.r = 0;
      m.color.g = 0;
      m.color.b = 0;
      m.color.a = 1.0;
      m.ns = invalidNamespace;
      m.id = currentInvalidId;
      ++currentInvalidId;
    }
    m.points.clear();
    for (unsigned int j = 0; j < twist.traj.poses.size(); ++j) {
      pt.x = twist.traj.poses[j].x;
      pt.y = twist.traj.poses[j].y;
      pt.z = 0;
      m.points.push_back(pt);
    }
    ma.markers.push_back(m);
  }
  marker_pub_.publish(ma);
}

void
DWBPublisher::publishLocalPlan(
  const std_msgs::Header & header,
  const dwb_msgs::Trajectory2D & traj)
{
  if (!publish_local_plan_) {return;}

  nav_msgs::Path path =
    nav_2d_utils::poses2DToPath(
      traj.poses, header.frame_id,
      header.stamp);

  if (local_pub_.getNumSubscribers() > 0) {
    local_pub_.publish(path);
  }
}

void
DWBPublisher::publishCostGrid(
  const costmap_2d::Costmap2DROS* costmap_ros,
  const std::vector<TrajectoryCritic::Ptr> critics)
{
  if (cost_grid_pc_pub_.getNumSubscribers() < 1) {return;}

  if (!publish_cost_grid_pc_) {return;}

  sensor_msgs::PointCloud2 cost_grid_pc;
  cost_grid_pc.header.frame_id = costmap_ros->getGlobalFrameID();
  cost_grid_pc.header.stamp = ros::Time::now();

  costmap_2d::Costmap2D * costmap = costmap_ros->getCostmap();
  double x_coord, y_coord;
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  std::vector<std::pair<std::string, std::vector<float>>> cost_channels;
  std::vector<float> total_cost(size_x * size_y, 0.0);

  for (TrajectoryCritic::Ptr critic : critics) {
    unsigned int channel_index = cost_channels.size();
    critic->addCriticVisualization(cost_channels);
    if (channel_index == cost_channels.size()) {
      // No channels were added, so skip to next critic
      continue;
    }
    double scale = critic->getScale();
    for (unsigned int i = 0; i < size_x * size_y; i++) {
      total_cost[i] += cost_channels[channel_index].second[i] * scale;
    }
  }

  cost_channels.push_back(std::make_pair("total_cost", total_cost));

  cost_grid_pc.width = size_x * size_y;
  cost_grid_pc.height = 1;
  cost_grid_pc.fields.resize(3 + cost_channels.size());  // x,y,z, + cost channels
  cost_grid_pc.is_dense = true;
  cost_grid_pc.is_bigendian = false;

  int offset = 0;
  for (size_t i = 0; i < cost_grid_pc.fields.size(); ++i, offset += 4) {
    cost_grid_pc.fields[i].offset = offset;
    cost_grid_pc.fields[i].count = 1;
    cost_grid_pc.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    if (i >= 3) {
      cost_grid_pc.fields[i].name = cost_channels[i - 3].first;
    }
  }

  cost_grid_pc.fields[0].name = "x";
  cost_grid_pc.fields[1].name = "y";
  cost_grid_pc.fields[2].name = "z";

  cost_grid_pc.point_step = offset;
  cost_grid_pc.row_step = cost_grid_pc.point_step * cost_grid_pc.width;
  cost_grid_pc.data.resize(cost_grid_pc.row_step * cost_grid_pc.height);

  std::vector<sensor_msgs::PointCloud2Iterator<float>> cost_grid_pc_iter;

  for (size_t i = 0; i < cost_grid_pc.fields.size(); ++i) {
    sensor_msgs::PointCloud2Iterator<float> iter(cost_grid_pc, cost_grid_pc.fields[i].name);
    cost_grid_pc_iter.push_back(iter);
  }

  unsigned int j = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      costmap->mapToWorld(cx, cy, x_coord, y_coord);
      *cost_grid_pc_iter[0] = x_coord;
      *cost_grid_pc_iter[1] = y_coord;
      *cost_grid_pc_iter[2] = 0.0;   // z value

      for (size_t i = 3; i < cost_grid_pc_iter.size(); ++i) {
        *cost_grid_pc_iter[i] = cost_channels[i - 3].second[j];
        ++cost_grid_pc_iter[i];
      }
      ++cost_grid_pc_iter[0];
      ++cost_grid_pc_iter[1];
      ++cost_grid_pc_iter[2];
      j++;
    }
  }

  cost_grid_pc_pub_.publish(cost_grid_pc);
}

void
DWBPublisher::publishGlobalPlan(const nav_2d_msgs::Path2D plan)
{
  publishGenericPlan(plan, global_pub_, publish_global_plan_);
}

void
DWBPublisher::publishTransformedPlan(const nav_2d_msgs::Path2D plan)
{
  publishGenericPlan(plan, transformed_pub_, publish_transformed_);
}

void
DWBPublisher::publishLocalPlan(const nav_2d_msgs::Path2D plan)
{
  publishGenericPlan(plan, local_pub_, publish_local_plan_);
}

void
DWBPublisher::publishGenericPlan(
  const nav_2d_msgs::Path2D plan,
  ros::Publisher & pub, bool flag)
{
  if (pub.getNumSubscribers() < 1) {return;}
  if (!flag) {return;}
  nav_msgs::Path path = nav_2d_utils::pathToPath(plan);
  pub.publish(path);
}

}  // namespace dwb_core
