cmake_minimum_required(VERSION 3.10)
project(control)

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

# Compile source files into a library
add_library(control_lib
  src/control_core.cpp
)

# Indicate to compiler where to search for header files
target_include_directories(control_lib
  PUBLIC
    include
)

# Add ROS2 dependencies required by package
ament_target_dependencies(control_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

# Create ROS2 node executable from source files
add_executable(control_node 
  src/control_node.cpp
)

# Link to the previously built library
target_link_libraries(control_node
  control_lib
)

# Add dependencies to the executable
ament_target_dependencies(control_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

# Install targets
install(TARGETS
  control_lib
  control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch directory if it exists
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()