cmake_minimum_required(VERSION 3.8)
project(graph_slam_wrapper)

#add_definitions(DGTSAM_POSE3_EXPMAP ON)
#add_definitions(DGTSAM_ROT3_EXPMAP ON)
#add_definitions(DGTSAM_USE_SYSTEM_EIGEN ON)
#add_definitions(DGTSAM_BUILD_WITH_MARCH_NATIVE OFF)
#add_definitions(DGTSAM_WITH_TBB OFF)
#set(DGTSAM_WITH_TBB OFF)
#DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_TBB=OFF
#set(DGTSAM_POSE3_EXPMAP ON)
#set(DGTSAM_ROT3_EXPMAP ON)
#set(DGTSAM_USE_SYSTEM_EIGEN ON)
#set(DGTSAM_BUILD_WITH_MARCH_NATIVE OFF)
#set(DGTSAM_WITH_TBB OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "Before GTSAM")

find_package(GTSAM REQUIRED)
#find_package(TBB REQUIRED)

# Manually specify if GTSAM_LIBRARIES is empty
if(NOT GTSAM_LIBRARIES)
	#find_library(GTSAM_LIB gtsam PATHS /opt/ros/humble/lib)
  set(GTSAM_LIBRARIES /usr/local/lib/cmake/GTSAM/../../../lib/libgtsam.so)
  message(STATUS "GTSAM_LIBRARIES set manually")
endif()

if(NOT GTSAM_LIBRARY_DIRS)
    set(GTSAM_LIBRARY_DIRS /usr/local/lib/cmake/GTSAM/../../../lib)
    message(STATUS "GTSAM_LIBRARY_DIRS set manually")
endif()

message(STATUS "GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR}")
message(STATUS "GTSAM_LIBRARY_DIRS: ${GTSAM_LIBRARY_DIRS}")
message(STATUS "GTSAM_LIBRARIES: ${GTSAM_LIBRARIES}")

include_directories(
        include
        ${GTSAM_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIRS}
)

link_directories(${GTSAM_LIBRARY_DIRS})
add_definitions(${GTSAM_DEFINITIONS})

add_executable(graph_slam_wrapper_node src/graph_slam_wrapper.cpp)
ament_target_dependencies(graph_slam_wrapper_node
        rclcpp
        nav_msgs
        geometry_msgs
        tf2
        tf2_geometry_msgs
#        GTSAM
)

target_link_libraries(
        graph_slam_wrapper_node ${GTSAM_LIBRARIES})

install(TARGETS
        graph_slam_wrapper_node
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
