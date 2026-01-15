cmake_minimum_required(VERSION 3.8)
project(navigation_2025)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# add_executable(node src/nav_node.py)
# ament_target_dependencies(my_nav_node rclcpp nav2_core nav2_util nav2_msgs nav2_behavior_tree geometry_msgs sensor_msgs tf2_ros)
install(
  DIRECTORY
    config launch maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
