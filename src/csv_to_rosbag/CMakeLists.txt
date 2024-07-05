cmake_minimum_required(VERSION 3.8)
project(csv_to_rosbag)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include our own headers
include_directories(include)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/csv_to_rosbag.cpp
  src/csv_reader.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  rosbag2_cpp
  sensor_msgs
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
