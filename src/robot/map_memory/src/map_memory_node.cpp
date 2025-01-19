#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory_node"),
  map_memory_(get_logger()),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_theta_(0.0),
  last_robot_x_(0.0),
  last_robot_y_(0.0)
{
  // Process all parameters
  processParameters();

  // Initialize the map memory core with parameters
  map_memory_.initMapMemory(
    resolution_,
    width_,
    height_,
    origin_
  );

  // Create subscribers
  local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_,
    10,
    std::bind(&MapMemoryNode::localCostmapCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Create publisher
  global_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_,
    10);

  // Create timer for publishing map
  timer_ = create_wall_timer(
    std::chrono::milliseconds(map_pub_rate_),
    std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::processParameters()
{
  // Declare and get parameters
  this->declare_parameter("local_costmap_topic", "local_costmap");
  this->declare_parameter("odom_topic", "odom");
  this->declare_parameter("map_topic", "map");
  this->declare_parameter("map_pub_rate", 1000);
  this->declare_parameter("update_distance", 1.0);
  this->declare_parameter("global_map.resolution", 0.05);
  this->declare_parameter("global_map.width", 1000);
  this->declare_parameter("global_map.height", 1000);
  this->declare_parameter("global_map.origin.position.x", 0.0);
  this->declare_parameter("global_map.origin.position.y", 0.0);
  this->declare_parameter("global_map.origin.orientation.w", 1.0);

  // Get parameters
  local_costmap_topic_ = this->get_parameter("local_costmap_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  map_pub_rate_ = this->get_parameter("map_pub_rate").as_int();
  update_distance_ = this->get_parameter("update_distance").as_double();
  resolution_ = this->get_parameter("global_map.resolution").as_double();
  width_ = this->get_parameter("global_map.width").as_int();
  height_ = this->get_parameter("global_map.height").as_int();

  // Set up origin pose
  origin_.position.x = this->get_parameter("global_map.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("global_map.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("global_map.origin.orientation.w").as_double();
}

void MapMemoryNode::localCostmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Calculate distance moved since last update
  double dx = robot_x_ - last_robot_x_;
  double dy = robot_y_ - last_robot_y_;
  double distance_moved = std::sqrt(dx * dx + dy * dy);

  // Update map if robot has moved sufficient distance
  if (distance_moved >= update_distance_) {
    map_memory_.updateMap(msg, robot_x_, robot_y_, robot_theta_);
    last_robot_x_ = robot_x_;
    last_robot_y_ = robot_y_;
  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_theta_ = quaternionToYaw(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
}

void MapMemoryNode::timerCallback()
{
  auto map_msg = map_memory_.getMapData();
  if (map_msg) {
    global_costmap_pub_->publish(*map_msg);
  }
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w)
{
  // Convert quaternion to yaw (rotation around Z axis)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}