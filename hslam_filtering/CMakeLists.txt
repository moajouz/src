cmake_minimum_required(VERSION 3.5)
project(hslam_filtering)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(python_cmake_module REQUIRED)

# Add C++ executables
add_executable(data_publisher src/data_publisher.cpp)
ament_target_dependencies(data_publisher rclcpp nav_msgs sensor_msgs)

# Install C++ executables
install(TARGETS
  data_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/data_control.py
  scripts/save_ukf_results.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ensure the Python scripts are executable
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME}")
install(FILES
  scripts/data_control.py
  scripts/save_ukf_results.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
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
