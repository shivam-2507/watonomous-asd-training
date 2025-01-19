#include "control_node.hpp"
#include <tf2/utils.h>

ControlNode::ControlNode()
    : Node("control_node"),
      control_(get_logger())
{
  // Process parameters
  processParameters();

  // Initialize control core
  control_.initControlCore(
      lookahead_distance_,
      max_steering_angle_,
      steering_gain_,
      linear_velocity_);

  // Create subscribers
  path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  // Create publisher
  cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10);

  // Create timer
  timer_ = create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&ControlNode::timerCallback, this));

  RCLCPP_INFO(get_logger(), "Control node initialized");
}

void ControlNode::processParameters()
{
  // Declare and get parameters
  this->declare_parameter("path_topic", "/path");
  this->declare_parameter("odom_topic", "/odom/filtered");
  this->declare_parameter("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter("control_period_ms", 100);
  this->declare_parameter("lookahead_distance", 1.5);
  this->declare_parameter("steering_gain", 1.5);
  this->declare_parameter("max_steering_angle", 0.5);
  this->declare_parameter("linear_velocity", 1.0);

  path_topic_ = this->get_parameter("path_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  control_period_ms_ = this->get_parameter("control_period_ms").as_int();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  steering_gain_ = this->get_parameter("steering_gain").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  linear_velocity_ = this->get_parameter("linear_velocity").as_double();

  RCLCPP_INFO(get_logger(), "Parameters loaded successfully");
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w)
{
  tf2::Quaternion q(x, y, z, w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received new path");
  control_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_theta_ = quaternionToYaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
}

void ControlNode::followPath()
{
  if (control_.isPathEmpty())
  {
    RCLCPP_WARN(get_logger(), "No path available");
    return;
  }

  geometry_msgs::msg::Twist cmd_vel = control_.calculateControlCommand(
      robot_x_, robot_y_, robot_theta_);

  cmd_vel_publisher_->publish(cmd_vel);
}

void ControlNode::timerCallback()
{
  followPath();
}