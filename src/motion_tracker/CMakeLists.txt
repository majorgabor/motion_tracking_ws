cmake_minimum_required(VERSION 3.8)
project(motion_tracker)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include our own headers
include_directories(include)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/motion_tracker.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
)

# Install our headers
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install our node and library
install(TARGETS
  ${PROJECT_NAME}_node 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
