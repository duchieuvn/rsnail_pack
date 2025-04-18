cmake_minimum_required(VERSION 3.8)
project(controller)  # Replace with your package name

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Add other dependencies here
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(
	include
)

# Add executable
add_executable(pure_pursuit src/pure_pursuit.cpp)  # Replace with your source file
ament_target_dependencies(pure_pursuit rclcpp)  # Link dependencies
ament_target_dependencies(pure_pursuit geometry_msgs)
ament_target_dependencies(pure_pursuit std_srvs)
ament_target_dependencies(pure_pursuit can_msgs)

# Install executable
install(TARGETS
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()

