cmake_minimum_required(VERSION 3.8)
project(elevator_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rclcpp_action     REQUIRED)   # for Nav2 action client
find_package(std_msgs          REQUIRED)
find_package(std_srvs          REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(nav2_msgs         REQUIRED)   # for NavigateToPose action
find_package(tf2               REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros           REQUIRED)
find_package(rcl_interfaces    REQUIRED)   # for SetParameters service

add_executable(elevator_supervisor_node
  src/ElevatorSupervisor.cpp
)

target_include_directories(elevator_supervisor_node PRIVATE
  include
)

ament_target_dependencies(elevator_supervisor_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  rcl_interfaces
)

install(TARGETS elevator_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch OPTIONAL)

ament_package()