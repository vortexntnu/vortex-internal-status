cmake_minimum_required(VERSION 3.8)
project(acoustic_modem)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)
find_package(nlohmann_json REQUIRED)


include_directories(include)

#set(LIB_NAME ${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  src/am_driver.cpp)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  serial_driver
  io_context
  nlohmann_json
)

# Install library and headers
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


#target_link_libraries()

#install(TARGETS
#am_node
#DESTINATION lib/${PROJECT_NAME}
#)


install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()
