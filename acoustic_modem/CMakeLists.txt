cmake_minimum_required(VERSION 3.8)
project(acoustic_modem)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
#find_package(spdlog REQUIRED)
#find_package(fmt REQUIRED)

include_directories(include)

add_executable(am_node
  src/am_driver.cpp
  src/am_ros.cpp
)


ament_target_dependencies(am_node
  rclcpp
  std_msgs
)


#target_link_libraries()

install(TARGETS
am_node
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
