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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <dwb_core/dwb_controller.hpp>
#include <dwb_core/exceptions.hpp>
#include <dwb_core/illegal_trajectory_tracker.hpp>
#include <dwb_msgs/CriticScore.h>
#include <nav_2d_msgs/Twist2D.h>
#include <nav_2d_utils/conversions.h>
#include <nav_2d_utils/tf_help.h>
#include <dwb_core/utils.hpp>
#include <dwb_core/controller_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <angles/angles.h>

using dwb_core::euclidean_distance;
using dwb_core::first_after_integrated_distance;
// using dwb_core::

// Register this controller as a nav_core plugin
PLUGINLIB_EXPORT_CLASS(dwb_core::DWBController, nav_core::BaseLocalPlanner)

namespace dwb_core
{

DWBController::DWBController()
: traj_gen_loader_("dwb_core", "dwb_core::TrajectoryGenerator"),
  critic_loader_("dwb_core", "dwb_core::TrajectoryCritic"),
  initialized_(false), goal_reached_(false), check_xy_(true)
{
}

void DWBController::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_) {
    ros::NodeHandle nh("~/" + name);
    ros::NodeHandle pnh_("~" + name);
    nh_ = pnh_;
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    dwb_plugin_name_ = name;

    std::string traj_generator_name;

    pnh_.param("odom_topic", odom_topic_, std::string("/odom"));
    pnh_.param("transform_tolerance", transform_tolerance_, 1.0);

    pnh_.param("goal_angular_vel_scaling_angle", goal_angular_vel_scaling_angle_, 30.0);
    pnh_.param("goal_angle_scaling_factor", goal_angle_scaling_factor_, 1.2);
    pnh_.param("rotate_to_goal_max_angular_vel", rotate_to_goal_max_angular_vel_, 0.5);
    pnh_.param("rotate_to_goal_min_angular_vel", rotate_to_goal_min_angular_vel_, 0.05);
    pnh_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
    pnh_.param("yaw_tolerance", yaw_tolerance_, 0.02);

    pnh_.param("prune_plan", prune_plan_, true);
    pnh_.param("prune_distance", prune_distance_, 2.0);
    pnh_.param("forward_prune_distance", forward_prune_distance_, 2.0);
    if (forward_prune_distance_ < 0.0) {
      ROS_WARN(
        "%s: ""Forward prune distance is negative, setting to max to search"
        " every point on path for the closest value.", controller_name_.c_str());
      forward_prune_distance_ = std::numeric_limits<double>::max();
    }
    pnh_.param("debug_trajectory_details", debug_trajectory_details_, false);
    pnh_.param("trajectory_generator_name", traj_generator_name, std::string("dwb_plugins::StandardTrajectoryGenerator"));
    pnh_.param(
      "short_circuit_trajectory_evaluation",
      short_circuit_trajectory_evaluation_, true);
    pnh_.param("shorten_transformed_plan", shorten_transformed_plan_, true);

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(odom_topic_);

    pub_ = std::make_unique<DWBPublisher>(nh, dwb_plugin_name_);
    pub_->initialize(nh);

    traj_generator_ = traj_gen_loader_.createUniqueInstance(traj_generator_name);
    traj_generator_->initialize(pnh_, dwb_plugin_name_);

    try {
      loadCritics();
    } catch (const std::exception & e) {
      ROS_ERROR("%s: ""Couldn't load critics! Caught exception: %s", controller_name_.c_str(), e.what());
      throw dwb_core::ControllerException(
              "Couldn't load critics! Caught exception: " +
              std::string(e.what()));
    }
    ROS_INFO("Created %s plugin.", controller_name_.c_str());
    initialized_ = true;
  }
}

std::string
DWBController::resolveCriticClassName(std::string base_name)
{
  if (base_name.find("Critic") == std::string::npos) {
    base_name = base_name + "Critic";
  }

  if (base_name.find("::") == std::string::npos) {
    for (unsigned int j = 0; j < default_critic_namespaces_.size(); j++) {
      std::string full_name = default_critic_namespaces_[j] + "::" + base_name;
      if (critic_loader_.isClassAvailable(full_name)) {
        return full_name;
      }
    }
  }
  return base_name;
}

void
DWBController::loadCritics()
{
  nh_.param("default_critic_namespaces", default_critic_namespaces_);
  if (default_critic_namespaces_.empty()) {
    default_critic_namespaces_.emplace_back("dwb_critics");
  }

  std::vector<std::string> critic_names;
  if (!nh_.getParam("critics", critic_names)) {
    ROS_WARN("%s: No critics found, using critics default.", controller_name_.c_str());
    std::vector<std::string> critic_names = {"ObstacleFootprint", "PathAlign", "PathDistPruned", "PathProgress"};
  }

  nh_.param("critics", critic_names);
  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string critic_plugin_name = critic_names[i];
    std::string plugin_class;

    plugin_class = resolveCriticClassName(critic_plugin_name);

    TrajectoryCritic::Ptr plugin = critic_loader_.createUniqueInstance(plugin_class);
    ROS_INFO(
      "%s: "
      "Using critic \"%s\" (%s)", controller_name_.c_str(), critic_plugin_name.c_str(), plugin_class.c_str());
    critics_.push_back(plugin);
    try {
      plugin->initialize(nh_, critic_plugin_name, dwb_plugin_name_, costmap_ros_);
    } catch (const std::exception & e) {
      ROS_ERROR("%s: ""Couldn't initialize critic plugin!", controller_name_.c_str());
      throw dwb_core::ControllerException(
              "Couldn't initialize critic plugin: " +
              std::string(e.what()));
    }
    ROS_INFO("%s: ""Critic plugin initialized", controller_name_.c_str());
  }
}

bool DWBController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_) {
    ROS_ERROR("%s: has not been initialized, please call initialize() before using this planner",
              controller_name_.c_str());
    return false;
  }

  auto path2d = nav_2d_utils::posesToPath2D(orig_global_plan);
  for (TrajectoryCritic::Ptr & critic : critics_) {
    critic->reset();
  }
  traj_generator_->reset();

  pub_->publishGlobalPlan(path2d);
  global_plan_ = path2d;

  goal_pose_.header.frame_id = global_plan_.header.frame_id;
  goal_pose_.pose = global_plan_.poses.back();

  goal_reached_ = false;
  return true;
}

bool DWBController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  std::shared_ptr<dwb_msgs::LocalPlanEvaluation> results = nullptr;
  if (pub_->shouldRecordEvaluation()) {
    results = std::make_shared<dwb_msgs::LocalPlanEvaluation>();
  }

  // Get robot pose respective "odom" frame id
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);

  // Get robot velocity
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

  goal_reached_ = false;
  double dist_to_goal;
  double angle_to_goal;

  try {
    nav_2d_msgs::Twist2DStamped cmd_vel2d = computeVelocityCommands(
      nav_2d_utils::poseStampedToPose2D(robot_pose),
      nav_2d_utils::twist3Dto2D(robot_vel_), results);
    pub_->publishEvaluation(results);
    cmd_vel = nav_2d_utils::twist2Dto3D(cmd_vel2d.velocity);
    return true;
  } catch (const dwb_core::ControllerTFError & e) {
    pub_->publishEvaluation(results);
  } catch (const dwb_core::InvalidPath & e) {
    pub_->publishEvaluation(results);
  } catch (const dwb_core::NoValidControl & e) {
    pub_->publishEvaluation(results);
  } catch (const dwb_core::ControllerException & e) {
    pub_->publishEvaluation(results);
  }
  return false;
}

bool DWBController::isGoalReached()
{
    if (goal_reached_)
    {
        check_xy_ = true;
        ROS_INFO("%s: ""GOAL Reached!", controller_name_.c_str());
        return true;
    }
    return false;
}

bool DWBController::isGoalReached(
  const nav_2d_msgs::Pose2DStamped & curr_robot_pose,
  nav_2d_msgs::Pose2DStamped & goal_pose,
  nav_2d_msgs::Pose2DStamped & transformed_end_pose)
{
  // When robot rotate in place check if recieve a new goal before done
  // rotate in place avoid robot get oscillation
  if (prev_goal_pose_.pose != goal_pose.pose) {
    check_xy_ = true;
  }
  prev_goal_pose_.pose = goal_pose.pose;
  
  double dx = curr_robot_pose.pose.x - transformed_end_pose.pose.x,
         dy = curr_robot_pose.pose.y - transformed_end_pose.pose.y;
  if (check_xy_) {
    if (dx * dx + dy * dy > xy_goal_tolerance_*xy_goal_tolerance_) {
      return false;
    }
    else {
      check_xy_ = false;
      return true;
    }
  } else {
    return true;
  }
}

void DWBController::prepareGlobalPlan(
  const nav_2d_msgs::Pose2DStamped & pose, nav_2d_msgs::Path2D & transformed_plan,
  nav_2d_msgs::Pose2DStamped & goal_pose,
  double &dist_to_goal,
  double &angle_to_goal,
  bool publish_plan)
{
  transformed_plan = transformGlobalPlan(pose, dist_to_goal, angle_to_goal);
  if (publish_plan) {
    pub_->publishTransformedPlan(transformed_plan);
  }
  if (!dwb_core::transformPose(
      tf_, costmap_ros_->getGlobalFrameID(), goal_pose_,
      goal_pose, transform_tolerance_)) {
    throw dwb_core::
          ControllerTFError("Unable to transform goal pose into global plan's frame");
  }
}

nav_2d_msgs::Twist2DStamped
DWBController::computeVelocityCommands(
  const nav_2d_msgs::Pose2DStamped & pose,
  const nav_2d_msgs::Twist2D & velocity,
  std::shared_ptr<dwb_msgs::LocalPlanEvaluation> & results)
{
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = ros::Time::now();
  }

  nav_2d_msgs::Path2D transformed_plan;
  nav_2d_msgs::Pose2DStamped transformed_end_pose;

  double dist_to_goal;
  double angle_to_goal;

  prepareGlobalPlan(pose,
                    transformed_plan,
                    transformed_end_pose,
                    dist_to_goal,
                    angle_to_goal);

  costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Return Value
  nav_2d_msgs::Twist2DStamped cmd_vel;
  cmd_vel.header.stamp = ros::Time::now();

  // Check if xy reached
  if (isGoalReached(pose, goal_pose_, transformed_end_pose)) {
    if (shouldRotateToGoalHeading(angle_to_goal)) {
      ROS_INFO("%s: ""Rotating to goal heading...", controller_name_.c_str());
      rotateToHeading(cmd_vel.velocity.x, cmd_vel.velocity.theta, angle_to_goal);
    }
    else {
      goal_reached_ = true;
      cmd_vel.velocity.x = 0.0;
      cmd_vel.velocity.theta = 0.0;
    }
    return cmd_vel;
  }

  for (TrajectoryCritic::Ptr & critic : critics_) {
    if (!critic->prepare(pose.pose, velocity, transformed_end_pose.pose, transformed_plan)) {
      ROS_WARN("%s: ""A scoring function failed to prepare", controller_name_.c_str());
    }
  }

  try {
    dwb_msgs::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results);

    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(cmd_vel.velocity);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, best.traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException & e) {
    nav_2d_msgs::Twist2D empty_cmd;
    dwb_msgs::Trajectory2D empty_traj;
    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr & critic : critics_) {
      critic->debrief(empty_cmd);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, empty_traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    throw dwb_core::NoValidControl(
            "Could not find a legal trajectory: " +
            std::string(e.what()));
  }
}

dwb_msgs::TrajectoryScore
DWBController::coreScoringAlgorithm(
  const geometry_msgs::Pose2D & pose,
  const nav_2d_msgs::Twist2D velocity,
  std::shared_ptr<dwb_msgs::LocalPlanEvaluation> & results)
{
  nav_2d_msgs::Twist2D twist;
  dwb_msgs::Trajectory2D traj;
  dwb_msgs::TrajectoryScore best, worst;
  best.total = -1;
  worst.total = -1;
  IllegalTrajectoryTracker tracker;

  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    twist = traj_generator_->nextTwist();
    traj = traj_generator_->generateTrajectory(pose, velocity, twist);

    try {
      dwb_msgs::TrajectoryScore score = scoreTrajectory(traj, best.total);
      tracker.addLegalTrajectory();
      if (results) {
        results->twists.push_back(score);
      }
      if (best.total < 0 || score.total < best.total) {
        best = score;
        if (results) {
          results->best_index = results->twists.size() - 1;
        }
      }
      if (worst.total < 0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1;
        }
      }
    } catch (const dwb_core::IllegalTrajectoryException & e) {
      if (results) {
        dwb_msgs::TrajectoryScore failed_score;
        failed_score.traj = traj;

        dwb_msgs::CriticScore cs;
        cs.name = e.getCriticName();
        cs.raw_score = -1.0;
        failed_score.scores.push_back(cs);
        failed_score.total = -1.0;
        results->twists.push_back(failed_score);
      }
      tracker.addIllegalTrajectory(e);
    }
  }

  if (best.total < 0) {
    if (debug_trajectory_details_) {
      ROS_ERROR("%s: ""%s", controller_name_.c_str(), tracker.getMessage().c_str());
      for (auto const & x : tracker.getPercentages()) {
        ROS_ERROR(
          "%s: ""%.2f: %10s/%s", controller_name_.c_str(), x.second,
          x.first.first.c_str(), x.first.second.c_str());
      }
    }
    throw NoLegalTrajectoriesException(tracker);
  }

  return best;
}

dwb_msgs::TrajectoryScore
DWBController::scoreTrajectory(
  const dwb_msgs::Trajectory2D & traj,
  double best_score)
{
  dwb_msgs::TrajectoryScore score;
  score.traj = traj;

  for (TrajectoryCritic::Ptr & critic : critics_) {
    dwb_msgs::CriticScore cs;
    cs.name = critic->getName();
    cs.scale = critic->getScale();

    if (cs.scale == 0.0) {
      score.scores.push_back(cs);
      continue;
    }

    double critic_score = critic->scoreTrajectory(traj);
    cs.raw_score = critic_score;
    score.scores.push_back(cs);
    score.total += critic_score * cs.scale;
    if (short_circuit_trajectory_evaluation_ && best_score > 0 && score.total > best_score) {
      // since we keep adding positives, once we are worse than the best, we will stay worse
      break;
    }
  }

  return score;
}

nav_2d_msgs::Path2D
DWBController::transformGlobalPlan(
  const nav_2d_msgs::Pose2DStamped & pose,
  double &dist_to_goal,
  double &angle_to_goal)
{
  if (global_plan_.poses.empty()) {
    throw dwb_core::InvalidPath("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::Pose2DStamped robot_pose;
  if (!dwb_core::transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw dwb_core::
          ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  double dx_ = global_plan_.poses.back().x - robot_pose.pose.x;
  double dy_ = global_plan_.poses.back().y - robot_pose.pose.y;
  dist_to_goal = std::hypot(dx_, dy_);
  angle_to_goal = angles::normalize_angle(global_plan_.poses.back().theta - robot_pose.pose.theta);

  // we'll discard points on the plan that are outside the local costmap
  costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double prune_dist = prune_distance_;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  if (prune_plan_) {
    transform_start_threshold = std::min(dist_threshold, prune_dist);
  } else {
    transform_start_threshold = dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the plan
  // near the robot (the end of the plan). This determines the amount of the
  // plan passed on to the critics
  double transform_end_threshold;
  double forward_prune_dist = forward_prune_distance_;
  if (shorten_transformed_plan_) {
    transform_end_threshold = std::min(dist_threshold, forward_prune_dist);
  } else {
    transform_end_threshold = dist_threshold;
  }

  // Find the first pose in the global plan that's further than forward prune distance
  // from the robot using integrated distance
  auto prune_point = first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), forward_prune_distance_);

  // Find the first pose in the plan (upto prune_point) that's less than transform_start_threshold
  // from the robot.
  auto transformation_begin = std::find_if(
    begin(global_plan_.poses), prune_point,
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose.pose, global_plan_pose) < transform_start_threshold;
    });

  // Find the first pose in the end of the plan that's further than transform_end_threshold
  // from the robot using integrated distance
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose.pose) > transform_end_threshold;
    });

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_2d_msgs::Path2D transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      nav_2d_msgs::Pose2DStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.pose = global_plan_pose;
      if (!dwb_core::transformPose(
        tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_)) {
        throw dwb_core::
          ControllerTFError("Unable to transform pose into global plan's frame");
      }
      return transformed_pose.pose;
    };

  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration.
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    pub_->publishGlobalPlan(global_plan_);
  }

  if (transformed_plan.poses.empty()) {
    throw dwb_core::InvalidPath("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

bool DWBController::shouldRotateToGoalHeading(double angle_to_goal)
{
  // Whether we should rotate robot to goal heading
  return fabs(angle_to_goal) >= yaw_tolerance_;
}

void DWBController::rotateToHeading(double & linear_vel, double & angular_vel,
                                    const double & angle_to_path)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  double angle_to_goal;
      
  if (std::abs(angle_to_path) < angles::from_degrees(goal_angular_vel_scaling_angle_)) {
      angle_to_goal = std::abs(angle_to_path) / goal_angle_scaling_factor_;
  } else {
      angle_to_goal = 1.0;
  }

  double rotate_to_goal_angular_vel = rotate_to_goal_max_angular_vel_;
  double unbounded_angular_vel = rotate_to_goal_angular_vel * angle_to_goal;

  if (unbounded_angular_vel < rotate_to_goal_min_angular_vel_) {
      rotate_to_goal_angular_vel = rotate_to_goal_min_angular_vel_;
  } else {
      rotate_to_goal_angular_vel = unbounded_angular_vel;
  }
  angular_vel = sign*clamp(rotate_to_goal_angular_vel,
                           rotate_to_goal_min_angular_vel_,
                           rotate_to_goal_max_angular_vel_);
}

}  // namespace dwb_core