cmake_minimum_required(VERSION 3.8)
project(acoustic_modem)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# just for debugging
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 # add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)


include_directories(include)

#set(LIB_NAME ${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  src/am_driver.cpp
  src/tdma_link.cpp
  src/tdma_manager.cpp
  src/am_driver_split.cpp
  )


  
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  serial_driver
  io_context
)

add_executable(am_base_node
  src/am_base_node.cpp
)
target_link_libraries(am_base_node ${PROJECT_NAME})
ament_target_dependencies(am_base_node rclcpp std_msgs)

add_executable(am_drone_node
  src/am_drone_node.cpp
)
target_link_libraries(am_drone_node ${PROJECT_NAME})
ament_target_dependencies(am_drone_node rclcpp std_msgs)

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
  TARGETS
    am_base_node
    am_drone_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()

