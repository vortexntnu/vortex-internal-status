cmake_minimum_required(VERSION 3.8)
project(ms5837_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ms5837_node
  src/ms5837.cpp
  src/ms5837_node.cpp
)

target_include_directories(ms5837_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ms5837_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS ms5837_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
