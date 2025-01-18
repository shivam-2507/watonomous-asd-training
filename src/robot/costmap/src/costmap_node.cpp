#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap_node"), costmap_(get_logger())
{
  // from yaml file and setps up the costmap
  processParameters();

  // init costmap
  costmap_.initCostmap(
    resolution_,
    width_,
    height_,
    origin_,
    inflation_radius_
  );

  // subscriber for laser scan data from node.hpp
  laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_topic_,
    10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1)
  );

  // publisher for costmap from node.hpp
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_,
    10
  );

  RCLCPP_INFO(get_logger(), "Costmap Node has been initialized");
}

void CostmapNode::processParameters()
{
  // from yaml file and setps up the costmap
  this->declare_parameter("laserscan_topic", "/lidar");
  this->declare_parameter("costmap_topic", "/costmap");
  this->declare_parameter("costmap.resolution", 0.40);
  this->declare_parameter("costmap.width", 120);
  this->declare_parameter("costmap.height", 120);
  this->declare_parameter("costmap.origin.position.x", -24.0);
  this->declare_parameter("costmap.origin.position.y", -24.0);
  this->declare_parameter("costmap.origin.orientation.w", 1.0);
  this->declare_parameter("costmap.inflation_radius", 1.5);

  // parameters
  laserscan_topic_ = this->get_parameter("laserscan_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  resolution_ = this->get_parameter("costmap.resolution").as_double();
  width_ = this->get_parameter("costmap.width").as_int();
  height_ = this->get_parameter("costmap.height").as_int();
  
  // default (boilerplate ig)
  origin_.position.x = this->get_parameter("costmap.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("costmap.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("costmap.origin.orientation.w").as_double();
  origin_.orientation.x = 0.0;
  origin_.orientation.y = 0.0;
  origin_.orientation.z = 0.0;

  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();

  // logging for docker log (irrelevant for this (GPTed))
  RCLCPP_INFO(get_logger(), "Laser scan topic: %s", laserscan_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Costmap topic: %s", costmap_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Resolution: %.2f", resolution_);
  RCLCPP_INFO(get_logger(), "Width: %d", width_);
  RCLCPP_INFO(get_logger(), "Height: %d", height_);
  RCLCPP_INFO(get_logger(), "Inflation radius: %.2f", inflation_radius_);
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
  // update--> new laser scan data
  costmap_.updateCostmap(msg);

  // publish it
  auto costmap_msg = costmap_.getCostmapData();
  costmap_pub_->publish(*costmap_msg);
}