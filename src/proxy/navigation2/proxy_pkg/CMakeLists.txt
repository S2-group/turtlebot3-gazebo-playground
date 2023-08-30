cmake_minimum_required(VERSION 3.5)
project(proxy_pkg)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # Replace with actual package name

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OdometryReduce.msg"
  DEPENDENCIES std_msgs nav_msgs
)

# Install
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
