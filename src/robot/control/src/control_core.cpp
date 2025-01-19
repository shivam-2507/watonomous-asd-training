#include "control_core.hpp"
#include <cmath>

namespace robot
{
  ControlCore::ControlCore(const rclcpp::Logger &logger)
      : logger_(logger)
  {
  }

  void ControlCore::initControlCore(
      double lookahead_distance,
      double max_steering_angle,
      double steering_gain,
      double linear_velocity)
  {
    lookahead_distance_ = lookahead_distance;
    max_steering_angle_ = max_steering_angle;
    steering_gain_ = steering_gain;
    linear_velocity_ = linear_velocity;
  }

  void ControlCore::updatePath(nav_msgs::msg::Path path)
  {
    path_ = path;
    RCLCPP_INFO(logger_, "Path updated with %zu points", path_.poses.size());
  }

  bool ControlCore::isPathEmpty()
  {
    return path_.poses.empty();
  }

  unsigned int ControlCore::findLookaheadPoint(double robot_x, double robot_y, double robot_theta)
  {
    unsigned int closest_point = 0;
    double min_distance = std::numeric_limits<double>::max();

    // Find the closest point first
    for (unsigned int i = 0; i < path_.poses.size(); ++i)
    {
      double dx = path_.poses[i].pose.position.x - robot_x;
      double dy = path_.poses[i].pose.position.y - robot_y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < min_distance)
      {
        min_distance = distance;
        closest_point = i;
      }
    }

    // Look ahead from the closest point
    unsigned int lookahead_point = closest_point;
    for (unsigned int i = closest_point; i < path_.poses.size(); ++i)
    {
      double dx = path_.poses[i].pose.position.x - robot_x;
      double dy = path_.poses[i].pose.position.y - robot_y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance >= lookahead_distance_)
      {
        lookahead_point = i;
        break;
      }
    }

    return lookahead_point;
  }

  geometry_msgs::msg::Twist ControlCore::calculateControlCommand(
      double robot_x, double robot_y, double robot_theta)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    if (isPathEmpty())
    {
      RCLCPP_WARN(logger_, "Path is empty, stopping robot");
      return cmd_vel;
    }

    // Find lookahead point
    unsigned int target_idx = findLookaheadPoint(robot_x, robot_y, robot_theta);

    // Calculate angle to target
    double target_x = path_.poses[target_idx].pose.position.x;
    double target_y = path_.poses[target_idx].pose.position.y;

    double dx = target_x - robot_x;
    double dy = target_y - robot_y;

    // Calculate angle between robot heading and target point
    double target_angle = std::atan2(dy, dx);
    double angle_diff = target_angle - robot_theta;

    // Normalize angle difference to [-pi, pi]
    while (angle_diff > M_PI)
      angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI)
      angle_diff += 2.0 * M_PI;

    // Calculate steering angle
    double steering_angle = steering_gain_ * angle_diff;

    // Clamp steering angle
    steering_angle = std::max(-max_steering_angle_,
                              std::min(max_steering_angle_, steering_angle));

    // Set command velocities
    cmd_vel.linear.x = linear_velocity_;
    cmd_vel.angular.z = steering_angle;

    return cmd_vel;
  }
}