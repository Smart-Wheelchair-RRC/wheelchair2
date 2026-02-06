cmake_minimum_required(VERSION 3.8)
project(wheelchair2_bringup)

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
find_package(PCL REQUIRED COMPONENTS common io)

# 2. Build the C++ Node
# Make sure your file is at: src/ground_detection_robust.cpp
add_executable(ground_detection_node src/ground_detection.cpp)

# Include PCL headers (system dependency)
target_include_directories(ground_detection_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

# Use ament_target_dependencies for ROS packages (Fixes the missing .hpp error)
ament_target_dependencies(ground_detection_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

# Link PCL libraries (system dependency) manually
target_link_libraries(ground_detection_node
  ${PCL_LIBRARIES}
)

# 3. Install the C++ Node
install(TARGETS ground_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Existing Installs ---

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install rviz files
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/invert_lidar.py
  scripts/ground_detection.py
  scripts/ground_detection_fast.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()