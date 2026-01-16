cmake_minimum_required(VERSION 3.8)
project(navigation_2025)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(otagg_vision_interfaces REQUIRED)

# Include directories
include_directories(include)

# Custom BT nodes plugin library
add_library(bt_custom_nodes_plugin SHARED
  src/bt_custom_nodes_plugin.cpp
)
ament_target_dependencies(bt_custom_nodes_plugin
  rclcpp
  nav2_behavior_tree
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  behaviortree_cpp_v3
  otagg_vision_interfaces
)
target_compile_definitions(bt_custom_nodes_plugin PRIVATE BT_PLUGIN_EXPORT)

# Install library
install(TARGETS bt_custom_nodes_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin XML
install(FILES bt_custom_nodes_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install config, launch, maps
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
