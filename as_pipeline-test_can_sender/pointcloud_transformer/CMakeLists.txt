cmake_minimum_required(VERSION 3.8)
project(pointcloud_transformer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Add the executable
add_executable(pointcloud_transformer_node src/pointcloud_transformer_node.cpp)

# Link dependencies
ament_target_dependencies(pointcloud_transformer_node 
  rclcpp 
  sensor_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  tf2_sensor_msgs
)

# Install the executable
install(TARGETS pointcloud_transformer_node
        DESTINATION lib/${PROJECT_NAME})

# Enable linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE) # Disable cpplint if not in a git repo
  ament_lint_auto_find_test_dependencies()
endif()

# Mark this package as an ament package
ament_package()

