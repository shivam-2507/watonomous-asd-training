#include "map_memory_core.hpp"

namespace robot
{

  MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger)
      : logger_(logger)
  {
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }

  void MapMemoryCore::initMapMemory(
      double resolution,
      int width,
      int height,
      geometry_msgs::msg::Pose origin)
  {
    // Initialize the global map
    global_map_->info.resolution = resolution;
    global_map_->info.width = width;
    global_map_->info.height = height;
    global_map_->info.origin = origin;

    // Initialize the data array with -1 (unknown)
    global_map_->data.resize(width * height, -1);

    RCLCPP_INFO(
        logger_,
        "Initialized global map with resolution: %f, width: %d, height: %d",
        resolution, width, height);
  }

  void MapMemoryCore::updateMap(
      nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap,
      double robot_x, double robot_y, double robot_theta)
  {
    if (!global_map_)
    {
      RCLCPP_ERROR(logger_, "Global map not initialized!");
      return;
    }

    // Get local map information
    int local_width = local_costmap->info.width;
    int local_height = local_costmap->info.height;
    double local_resolution = local_costmap->info.resolution;

    // For each point in the local costmap
    for (int ly = 0; ly < local_height; ly++)
    {
      for (int lx = 0; lx < local_width; lx++)
      {
        // Get the occupancy value from local map
        int local_index = ly * local_width + lx;
        int occupancy = local_costmap->data[local_index];

        // Convert local map coordinates to world coordinates
        double world_x = local_costmap->info.origin.position.x +
                         (lx + 0.5) * local_resolution;
        double world_y = local_costmap->info.origin.position.y +
                         (ly + 0.5) * local_resolution;

        // Convert world coordinates to global map coordinates
        int mx, my;
        if (robotToMap(world_x, world_y, mx, my))
        {
          // Update the global map
          int global_index = my * global_map_->info.width + mx;
          if (global_index >= 0 &&
              global_index < static_cast<int>(global_map_->data.size()))
          {
            // Only update if the new information is more certain
            if (global_map_->data[global_index] == -1 ||
                std::abs(occupancy - 50) > std::abs(global_map_->data[global_index] - 50))
            {
              global_map_->data[global_index] = occupancy;
            }
          }
        }
      }
    }
  }

  bool MapMemoryCore::robotToMap(double rx, double ry, int &mx, int &my)
  {
    // Convert world coordinates to map coordinates
    mx = static_cast<int>(
        (rx - global_map_->info.origin.position.x) /
        global_map_->info.resolution);
    my = static_cast<int>(
        (ry - global_map_->info.origin.position.y) /
        global_map_->info.resolution);

    // Check if the point is within map bounds
    return (mx >= 0 && mx < static_cast<int>(global_map_->info.width) &&
            my >= 0 && my < static_cast<int>(global_map_->info.height));
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getMapData() const
  {
    return global_map_;
  }

} // namespace robot