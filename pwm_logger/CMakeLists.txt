find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pwm_csv_logger src/pwm_csv_logger.cpp)
ament_target_dependencies(pwm_csv_logger rclcpp std_msgs)

install(TARGETS
  pwm_csv_logger
  DESTINATION lib/${PROJECT_NAME}
)
