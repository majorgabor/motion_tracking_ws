cmake_minimum_required(VERSION 3.8)
project(socket_to_ros_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include our own headers
include_directories(include)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/udp_socket_server.cpp
  src/signal_handler.cpp
  src/publisher_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  sensor_msgs
)

# Install our headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install our node and library
install(TARGETS
  ${PROJECT_NAME}_node 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
