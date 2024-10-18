#include <dwb_core/goal_checker.hpp>
#include <angles/angles.h>
#include <dwb_core/utils.hpp>

namespace dwb_core
{

GoalChecker::GoalChecker(std::string name, ros::NodeHandle &nh):
  check_xy_(false)
{
  controller_name_ = name;
  initialize(nh);
}

void GoalChecker::initialize(ros::NodeHandle &nh)
{
  nh.param("controller_frequency", controller_frequency_, 15.0);
  control_duration_ = 1 / controller_frequency_;
  nh.param(plugin_name_ + "/goal_angular_vel_scaling_angle", goal_angular_vel_scaling_angle_, 30.0);
  nh.param(plugin_name_ + "/goal_angle_scaling_factor", goal_angle_scaling_factor_, 1.2);
  nh.param(plugin_name_ + "/rotate_to_goal_max_angular_vel", rotate_to_goal_max_angular_vel_, 0.5);
  nh.param(plugin_name_ + "/rotate_to_goal_min_angular_vel", rotate_to_goal_min_angular_vel_, 0.05);
  nh.param(plugin_name_ + "/xy_goal_tolerance", xy_goal_tolerance_, 0.1);
  nh.param(plugin_name_ + "/yaw_tolerance", yaw_tolerance_, 0.02);
}

void GoalChecker::reset()
{
  check_xy_ = true;
}

bool GoalChecker::isGoalReached(
  const nav_2d_msgs::Pose2DStamped & curr_robot_pose,
  nav_2d_msgs::Pose2DStamped & goal_pose,
  nav_2d_msgs::Pose2DStamped & transformed_end_pose,
  nav_2d_msgs::Twist2DStamped & cmd_vel,
  double angle_to_goal, bool &goal_reached)
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
    if (hypot_sq(dx, dy) > xy_goal_tolerance_*xy_goal_tolerance_) {
      return false;
    }
    else {
      check_xy_ = false;
    }
  }
  if (shouldRotateToGoalHeading(angle_to_goal)) {
    ROS_INFO("%s: ""Rotating to goal heading...", controller_name_.c_str());
    rotateToHeading(cmd_vel, angle_to_goal);
  }
  else {
    goal_reached = true;
    cmd_vel.velocity.x = 0.0;
    cmd_vel.velocity.theta = 0.0;
  }
  return true;
}

bool GoalChecker::shouldRotateToGoalHeading(double angle_to_goal)
{
  // Whether we should rotate robot to goal heading
  return fabs(angle_to_goal) >= yaw_tolerance_;
}

void GoalChecker::rotateToHeading(
  nav_2d_msgs::Twist2DStamped & cmd_vel,
  const double & angle_to_path)
{
  // Rotate in place using max angular velocity / acceleration possible
  cmd_vel.velocity.x = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  double angle_to_goal;

  bool is_stopped = fabs(angle_to_path) <= yaw_tolerance_;
      
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
  cmd_vel.velocity.theta = sign*clamp(rotate_to_goal_angular_vel,
                                      rotate_to_goal_min_angular_vel_,
                                      rotate_to_goal_max_angular_vel_);
}

} // namespace dwb_core
