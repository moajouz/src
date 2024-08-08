cmake_minimum_required(VERSION 3.5)
project(hslam_filtering)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(robot_localization REQUIRED)
 
# Add executable for data publisher
add_executable(data_publisher src/data_publisher.cpp)
ament_target_dependencies(data_publisher rclcpp nav_msgs builtin_interfaces sensor_msgs)

# Add executable for data publisher
add_executable(get_timestamp src/get_timestamp.cpp)
ament_target_dependencies(get_timestamp rclcpp nav_msgs builtin_interfaces std_msgs)


# Install executables
install(TARGETS
  data_publisher
  get_timestamp
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
