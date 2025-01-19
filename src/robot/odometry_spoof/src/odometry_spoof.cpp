#include "odometry_spoof_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>

using namespace std::chrono_literals;

OdometrySpoofNode::OdometrySpoofNode()
    : Node("odometry_spoof_node"),
      has_last_transform_(false)
{
  // Initialize tf buffer with clock
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize publisher for odometry data
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/robot", 10);

  // Create timer for periodic updates (10Hz)
  timer_ = this->create_wall_timer(
      100ms,
      std::bind(&OdometrySpoofNode::timerCallback, this));
}

void OdometrySpoofNode::timerCallback()
{
  // Define source and target frames for transform lookup
  const auto base_frame = "robot_base";
  const auto world_frame = "world";

  // Get the latest transform
  geometry_msgs::msg::TransformStamped current_transform;
  try
  {
    current_transform = tf_buffer_->lookupTransform(
        world_frame,
        base_frame,
        tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000, // Throttle period in ms
        "Transform lookup failed: %s", ex.what());
    return;
  }

  // Initialize odometry message
  auto odom = nav_msgs::msg::Odometry();

  // Set header information
  odom.header = current_transform.header;
  odom.header.frame_id = world_frame;
  odom.child_frame_id = base_frame;

  // Copy position data
  odom.pose.pose.position.x = current_transform.transform.translation.x;
  odom.pose.pose.position.y = current_transform.transform.translation.y;
  odom.pose.pose.position.z = current_transform.transform.translation.z;
  odom.pose.pose.orientation = current_transform.transform.rotation;

  // Initialize velocities to zero
  odom.twist.twist = geometry_msgs::msg::Twist();

  // Calculate velocities if we have a previous transform
  if (has_last_transform_)
  {
    // Get time delta
    const double delta_t = (current_transform.header.stamp - last_time_).seconds();

    if (delta_t > 0.0)
    {
      // Current position as tf2 vector
      tf2::Vector3 curr_pos(
          current_transform.transform.translation.x,
          current_transform.transform.translation.y,
          current_transform.transform.translation.z);

      // Calculate linear velocity
      tf2::Vector3 linear_vel = (curr_pos - last_position_) / delta_t;
      odom.twist.twist.linear.x = linear_vel.x();
      odom.twist.twist.linear.y = linear_vel.y();
      odom.twist.twist.linear.z = linear_vel.z();

      // Current orientation
      tf2::Quaternion curr_quat;
      tf2::fromMsg(current_transform.transform.rotation, curr_quat);

      // Calculate angular velocity using quaternion difference
      tf2::Quaternion delta_quat = last_orientation_.inverse() * curr_quat;

      // Convert to RPY
      double delta_roll, delta_pitch, delta_yaw;
      tf2::Matrix3x3(delta_quat).getRPY(delta_roll, delta_pitch, delta_yaw);

      // Set angular velocities
      odom.twist.twist.angular.x = delta_roll / delta_t;
      odom.twist.twist.angular.y = delta_pitch / delta_t;
      odom.twist.twist.angular.z = delta_yaw / delta_t;
    }
  }

  // Store current transform data for next iteration
  last_time_ = current_transform.header.stamp;
  tf2::fromMsg(current_transform.transform.translation, last_position_);
  tf2::fromMsg(current_transform.transform.rotation, last_orientation_);
  has_last_transform_ = true;

  // Publish the odometry message
  odom_pub_->publish(odom);
}

#include "odometry_spoof_node.hpp"
#include <memory>

int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create and spin node
  auto node = std::make_shared<OdometrySpoofNode>();
  rclcpp::spin(node);

  // Clean shutdown
  rclcpp::shutdown();
  return 0;
}