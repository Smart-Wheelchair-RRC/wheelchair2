cmake_minimum_required(VERSION 3.8)
project(wheelchair2_bringup)

# Default to C++17 for PCL compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Specifically request PCL components
find_package(PCL REQUIRED COMPONENTS common io segmentation filters)

# 2. Build the C++ Nodes

# --- REUSE PLANES NODE (The advanced one) ---
add_executable(ground_detection_node_reuse_planes src/ground_detection_reuse_planes_grid.cpp)
target_include_directories(ground_detection_node_reuse_planes PUBLIC 
  /usr/include/pcl-1.12
  /usr/include/eigen3
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ground_detection_node_reuse_planes ${PCL_LIBRARIES})
ament_target_dependencies(ground_detection_node_reuse_planes
  rclcpp sensor_msgs nav_msgs tf2_ros tf2_sensor_msgs pcl_conversions pcl_ros
)

# --- STANDARD GROUND DETECTION NODE (Added back) ---
add_executable(ground_detection_node src/ground_detection.cpp)
target_include_directories(ground_detection_node PUBLIC 
  /usr/include/pcl-1.12
  /usr/include/eigen3
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ground_detection_node ${PCL_LIBRARIES})
ament_target_dependencies(ground_detection_node
  rclcpp sensor_msgs nav_msgs tf2_ros tf2_sensor_msgs pcl_conversions pcl_ros
)

# 3. Install
install(TARGETS 
  ground_detection_node 
  ground_detection_node_reuse_planes
  DESTINATION lib/${PROJECT_NAME}
)

# Standard ROS 2 Installs
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/invert_lidar.py
  scripts/ground_detection.py
  scripts/ground_detection_fast.py
  scripts/box_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()