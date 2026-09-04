cmake_minimum_required(VERSION 3.8)
project(wheelchair2_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_library(wheelchair2_bt_plugins_node SHARED
  src/plugins.cpp
)

target_include_directories(wheelchair2_bt_plugins_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(wheelchair2_bt_plugins_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav2_msgs
  behaviortree_cpp_v3
)

ament_export_libraries(wheelchair2_bt_plugins_node)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs std_msgs nav2_msgs behaviortree_cpp_v3)

install(TARGETS wheelchair2_bt_plugins_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()