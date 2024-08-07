cmake_minimum_required(VERSION 3.5)
project(fusion)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)

# Add executable for GPSPublisher
add_executable(gps_publisher src/gps_publisher.cpp)
ament_target_dependencies(gps_publisher rclcpp sensor_msgs)

# Add executable for HSLAMPublisher
add_executable(hslam_publisher src/hslam_publisher.cpp)
ament_target_dependencies(hslam_publisher rclcpp nav_msgs)

# Add executable for MixedDataPublisher
add_executable(mixed_data_publisher src/mixed_data_publisher.cpp)
ament_target_dependencies(mixed_data_publisher rclcpp nav_msgs sensor_msgs)

# Add executable for MixedDataPublisher odom
add_executable(mixed_data_publisher_odom src/mixed_data_publisher_odom.cpp)
ament_target_dependencies(mixed_data_publisher_odom rclcpp nav_msgs sensor_msgs)

# Add executable for MixedDataPublisher odom with timestamp corrolation
add_executable(mixed_data_publisher_odom_times_corrolation src/mixed_data_publisher_odom_times_corrolation.cpp)
ament_target_dependencies(mixed_data_publisher_odom_times_corrolation rclcpp nav_msgs sensor_msgs)

# Install executables
install(TARGETS
  gps_publisher
  hslam_publisher
  mixed_data_publisher  # Add the new executable here
  mixed_data_publisher_odom
  mixed_data_publisher_odom_times_corrolation 
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and configuration files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
