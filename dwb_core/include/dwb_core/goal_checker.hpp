#ifndef DWB_CORE_GOAL_CHECKER_HPP_
#define DWB_CORE_GOAL_CHECKER_HPP_

#include <nav_2d_msgs/Pose2DStamped.h>
#include <nav_2d_msgs/Twist2DStamped.h>
#include <ros/ros.h>

namespace dwb_core
{

class GoalChecker
{
public:    
    GoalChecker(std::string name, ros::NodeHandle &nh);
    ~GoalChecker() {};

    void initialize(ros::NodeHandle &nh);

    /**
     * @brief Check if robot reaches xy
     * @return If true, check yaw -> True if yaw reached.
     */
    bool isGoalReached(
        const nav_2d_msgs::Pose2DStamped & curr_robot_pose,
        nav_2d_msgs::Pose2DStamped & goal_pose,
        nav_2d_msgs::Pose2DStamped & transformed_end_pose,
        nav_2d_msgs::Twist2DStamped & cmd_vel,
        double angle_to_goal, bool &goal_reached);
    
    void reset();

    /**
     * @brief Whether robot should rotate to final goal orientation
     * @param target_pose current lookahead point
     * @return Whether should rotate to goal heading
     */
    bool shouldRotateToGoalHeading(double angle_to_goal);

    void rotateToHeading(
        nav_2d_msgs::Twist2DStamped & cmd_vel,
        const double & angle_to_path);

private:
    bool check_xy_;
    std::string controller_name_;
    std::string plugin_name_ = "GoalChecker";
    nav_2d_msgs::Pose2DStamped prev_goal_pose_;
  
    // Params for rotate to goal
    double control_duration_;
    double controller_frequency_;
    double goal_angular_vel_scaling_angle_;
    double goal_angle_scaling_factor_;
    double rotate_to_goal_max_angular_vel_;
    double rotate_to_goal_min_angular_vel_;
    double xy_goal_tolerance_;
    double yaw_tolerance_;
};

} // namespace dwb_core

#endif // DWB_CORE_GOAL_CHECKER_HPP_