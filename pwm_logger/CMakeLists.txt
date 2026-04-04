cmake_minimum_required(VERSION 3.8)
project(pwm_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pwm_csv_logger src/pwm_csv_logger.cpp)
ament_target_dependencies(pwm_csv_logger rclcpp std_msgs)

install(TARGETS
  pwm_csv_logger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
