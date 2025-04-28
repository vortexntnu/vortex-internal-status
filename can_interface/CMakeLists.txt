cmake_minimum_required(VERSION 3.8)
project(can_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

add_executable(can_interface_node
  src/can_interface_driver.c
  src/can_interface_ros.cpp
)

ament_target_dependencies(can_interface_node
  rclcpp
  std_msgs
  spdlog
  fmt
  sensor_msgs
)


target_link_libraries(can_interface_node fmt::fmt)

install(TARGETS
can_interface_node
DESTINATION lib/${PROJECT_NAME}
)



install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
