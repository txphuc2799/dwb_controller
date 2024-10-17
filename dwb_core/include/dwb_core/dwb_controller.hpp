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

#ifndef DWB_CORE__DWB_CONTROLLER_HPP_
#define DWB_CORE__DWB_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <dwb_core/publisher.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <dwb_core/trajectory_generator.hpp>
#include <nav_2d_msgs/Pose2DStamped.h>
#include <nav_2d_msgs/Twist2DStamped.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_ros/buffer.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwb_core/footprint_collision_checker.hpp>

namespace dwb_core
{

/**
 * @class DWBController
 * @brief Plugin-based flexible controller
 */
class DWBController : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief Constructor that brings up pluginlib loaders
   */
  DWBController();

  ~DWBController() {}

  /**
   * @brief Initializes the plugin
   * @param name The name of the instance
   * @param tf Pointer to a tf buffer
   * @param costmap_ros Cost map representing occupied and free space
   */
  void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
  * @brief Set the plan that the local planner is following
  * @param orig_global_plan The plan to pass to the local planner
  * @return True if the plan was updated successfully, false otherwise
  */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
  * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
  * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
  * @return True if a valid trajectory was found, false otherwise
  */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
  * @brief  Check if the goal pose has been achieved
  * 
  * The actual check is performed in computeVelocityCommands(). 
  * Only the status flag is checked here.
  * @return True if achieved, false otherwise
  */
  bool isGoalReached();

  /**
   * @brief Check if robot reaches xy
   * @return If true, check yaw -> True if yaw reached.
   */
  bool isGoalReached(
    const nav_2d_msgs::Pose2DStamped & curr_robot_pose,
    nav_2d_msgs::Pose2DStamped & goal_pose,
    nav_2d_msgs::Pose2DStamped & transformed_end_pose);

  /**
   * @brief Score a given command. Can be used for testing.
   *
   * Given a trajectory, calculate the score where lower scores are better.
   * If the given (positive) score exceeds the best_score, calculation may be cut short, as the
   * score can only go up from there.
   *
   * @param traj Trajectory to check
   * @param best_score If positive, the threshold for early termination
   * @return The full scoring of the input trajectory
   */
  virtual dwb_msgs::TrajectoryScore scoreTrajectory(
    const dwb_msgs::Trajectory2D & traj,
    double best_score = -1);

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param results   Output param, if not NULL, will be filled in with full evaluation results
   * @return          Best command
   */
  virtual bool computeVelocityCommands(
    const nav_2d_msgs::Pose2DStamped & pose,
    const nav_2d_msgs::Twist2D & velocity,
    nav_2d_msgs::Twist2DStamped & vel_out,
    std::shared_ptr<dwb_msgs::LocalPlanEvaluation> & results);

  /**
   * @brief Checks if rotation is safe
   * @param cmd_vel Velocity to check over
   * @param angular_distance_to_heading Angular distance to heading requested
   * @param pose Starting pose of robot
   */
  bool isCollisionFree(
    double & linear_vel, double & angular_vel,
    bool is_stopped,
    const nav_2d_msgs::Pose2DStamped & pose);

protected:
  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param target_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(double angle_to_goal);

  bool rotateToHeading(
    double & linear_vel,
    double & angular_vel,
    const double & angle_to_path,
    const nav_2d_msgs::Pose2DStamped & robot_pose);
  
  /**
   * @brief Helper method for two common operations for the operating on the global_plan
   *
   * Transforms the global plan (stored in global_plan_) relative to the pose and saves it in
   * transformed_plan and possibly publishes it. Then it takes the last pose and transforms it
   * to match the local costmap's frame
   */
  void prepareGlobalPlan(
    const nav_2d_msgs::Pose2DStamped & pose, nav_2d_msgs::Path2D & transformed_plan,
    nav_2d_msgs::Pose2DStamped & goal_pose,
    double &angle_to_goal,
    bool publish_plan = true);

  /**
   * @brief Iterate through all the twists and find the best one
   */
  virtual dwb_msgs::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::Pose2D & pose,
    const nav_2d_msgs::Twist2D velocity,
    std::shared_ptr<dwb_msgs::LocalPlanEvaluation> & results);

  /**
   * @brief Transforms global plan into same frame as pose, clips far away poses and possibly prunes passed poses
   *
   * Three key operations
   * 1) Transforms global plan into frame of the given pose
   * 2) Only returns poses that are near the robot, i.e. whether they are likely on the local costmap
   * 3) If prune_plan_ is true, it will remove all points that we've already passed from both the transformed plan
   *     and the saved global_plan_. Technically, it iterates to a pose on the path that is within prune_distance_
   *     of the robot and erases all poses before that.
   *
   * Additionally, shorten_transformed_plan_ determines whether we will pass the full plan all
   * the way to the nav goal on to the critics or just a subset of the plan near the robot.
   * True means pass just a subset. This gives DWB less discretion to decide how it gets to the
   * nav goal. Instead it is encouraged to try to get on to the path generated by the global planner.
   */
  virtual nav_2d_msgs::Path2D transformGlobalPlan(
    const nav_2d_msgs::Pose2DStamped & pose,
    double &angle_to_goal);

  nav_2d_msgs::Path2D global_plan_;  ///< Saved Global Plan
  bool prune_plan_;
  double prune_distance_;
  bool debug_trajectory_details_;
  double transform_tolerance_;
  bool shorten_transformed_plan_;
  double forward_prune_distance_;
  std::string odom_topic_;
  
  /**
   * @brief try to resolve a possibly shortened critic name with the default namespaces and the suffix "Critic"
   *
   * @param base_name The name of the critic as read in from the parameter server
   * @return Our attempted resolution of the name, with namespace prepended and/or the suffix Critic appended
   */
  std::string resolveCriticClassName(std::string base_name);

  /**
   * @brief Load the critic parameters from the namespace
   * @param name The namespace of this planner.
   */
  virtual void loadCritics();

  ros::NodeHandle nh_;
  std::string controller_name_ = "DWBController";

  tf2_ros::Buffer* tf_; //!< pointer to tf buffer
  std::string plugin_name_;
  costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
  costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  base_local_planner::OdometryHelperRos odom_helper_; //!< Provides an interface to receive the current velocity from the robot
  geometry_msgs::Twist robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
  nav_2d_msgs::Pose2DStamped goal_pose_;
  nav_2d_msgs::Pose2DStamped prev_goal_pose_;

  std::unique_ptr<DWBPublisher> pub_;
  std::vector<std::string> default_critic_namespaces_;

  // Plugin handling
  pluginlib::ClassLoader<TrajectoryGenerator> traj_gen_loader_;
  TrajectoryGenerator::Ptr traj_generator_;

  pluginlib::ClassLoader<TrajectoryCritic> critic_loader_;
  std::vector<TrajectoryCritic::Ptr> critics_;
  std::unique_ptr<FootprintCollisionChecker<costmap_2d::Costmap2D *>>
  collision_checker_;

  std::string dwb_plugin_name_;

  bool short_circuit_trajectory_evaluation_;
  bool initialized_;
  bool goal_reached_;
  bool check_xy_;
  
  // Params for rotate to goal
  bool use_collision_detection_;
  double control_duration_;
  double controller_frequency_;
  double simulate_ahead_time_;
  double goal_angular_vel_scaling_angle_;
  double goal_angle_scaling_factor_;
  double rotate_to_goal_max_angular_vel_;
  double rotate_to_goal_min_angular_vel_;
  double xy_goal_tolerance_;
  double yaw_tolerance_;
};

}  // namespace dwb_core

#endif  // DWB_CORE__DWCONTROLLERER_HPP_
