cmake_minimum_required(VERSION 3.8)
project(acoustics_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(acoustics_interface
  src/acoustics_interface_driver.cpp
  src/acoustics_interface_ros.cpp
)

ament_target_dependencies(acoustics_interface
  rclcpp
  std_msgs
)


target_link_libraries(acoustics_interface)

install(TARGETS

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
