cmake_minimum_required(VERSION 3.10)
project(costmap VERSION 1.0.0 DESCRIPTION "Advanced Costmap Implementation for Robot Navigation")

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Compile core library
add_library(${PROJECT_NAME}_core
    src/costmap_core.cpp
)

# Set include directories for core library
target_include_directories(${PROJECT_NAME}_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Add dependencies for core library
ament_target_dependencies(${PROJECT_NAME}_core
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
)

# Create and link the node executable
add_executable(${PROJECT_NAME}_node
    src/costmap_node.cpp
)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_core
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
)

# Install targets
install(
    TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# Install launch and config directories if they exist
install(
    DIRECTORY
        launch
        config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Export targets
install(
    EXPORT export_${PROJECT_NAME}
    FILE export_${PROJECT_NAME}.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)

ament_package()