#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger &logger)
: logger_(logger)
{
  costmap_data_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
}

void CostmapCore::initCostmap(
  double resolution,
  int width,
  int height,
  geometry_msgs::msg::Pose origin,
  double inflation_radius)
{
  // inflation parameters
  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius / resolution));

  // costmap message
  costmap_data_->header.frame_id = "map";
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;
  costmap_data_->info.origin = origin;

  // init costmap data array
  costmap_data_->data.resize(width * height, 0);

  RCLCPP_INFO(logger_, "Costmap initialized with size %dx%d and resolution %.2f",
              width, height, resolution);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const
{
  // clear array of costmap costmap data (should this be with initCostmap?) --> someone checkup!
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  // update costmap header timestamp ()
  costmap_data_->header.stamp = laserscan->header.stamp;

  const double angle_min = laserscan->angle_min;
  const double angle_increment = laserscan->angle_increment;
  const double max_range = laserscan->range_max;

  // process each laser scan reading
  for (size_t i = 0; i < laserscan->ranges.size(); ++i)
  {
    double range = laserscan->ranges[i];
    
    // skip invalid readings (from GPT) --> check if this is correct
    if (!std::isfinite(range) || range < laserscan->range_min || range > max_range)
    {
      continue;
    }

    double angle = angle_min + (i * angle_increment);
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // polar to grid coordinates
    int grid_x = static_cast<int>((x - costmap_data_->info.origin.position.x) / 
                                  costmap_data_->info.resolution);
    int grid_y = static_cast<int>((y - costmap_data_->info.origin.position.y) / 
                                  costmap_data_->info.resolution);

    // check if the point is within grid bounds
    if (grid_x >= 0 && grid_x < static_cast<int>(costmap_data_->info.width) &&
        grid_y >= 0 && grid_y < static_cast<int>(costmap_data_->info.height))
    {
      // mark obstacle and inflate it
      int index = grid_y * costmap_data_->info.width + grid_x;
      costmap_data_->data[index] = 100;  // Mark as occupied
      inflateObstacle(grid_x, grid_y);
    }
  }
}

void CostmapCore::inflateObstacle(int origin_x, int origin_y) const
{
    // iterate over cells in inflation radius in array
  for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx)
  {
    for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy)
    {
      int x = origin_x + dx;
      int y = origin_y + dy;

        // must make bound checking a function of itself
      if (x >= 0 && x < static_cast<int>(costmap_data_->info.width) &&
          y >= 0 && y < static_cast<int>(costmap_data_->info.height))
      {
        // distance
        double distance = std::hypot(dx * costmap_data_->info.resolution,
                                   dy * costmap_data_->info.resolution);

        // only inflate if within inflation radius
        if (distance <= inflation_radius_)
        {
          int index = y * costmap_data_->info.width + x;
          // calculate cost based on distance (linear decay) --> check if right formula (dont think its linear)
          int cost = static_cast<int>((1.0 - distance / inflation_radius_) * 99);
          // set cost to max of current cost and new cost
          costmap_data_->data[index] = std::max(static_cast<int8_t>(cost),
                                              costmap_data_->data[index]);
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const
{
  return costmap_data_;
}

}  // namespace robot