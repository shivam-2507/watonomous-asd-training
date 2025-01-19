cmake_minimum_required(VERSION 3.10)
project(map_memory)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Create the map memory core library
add_library(map_memory_core
  src/map_memory_core.cpp
)

# Set include directories for the core library
target_include_directories(map_memory_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Add dependencies for the core library
ament_target_dependencies(map_memory_core
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

# Create the ROS2 node executable
add_executable(map_memory_node 
  src/map_memory_node.cpp
)

# Link the node with the core library
target_link_libraries(map_memory_node
  map_memory_core
)

# Add dependencies for the node
ament_target_dependencies(map_memory_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

# Install targets
install(
  TARGETS map_memory_core map_memory_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Export targets
install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

ament_export_include_directories(include)
ament_export_libraries(map_memory_core)

ament_package()