cmake_minimum_required(VERSION 3.5)

project(simulation_2025)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install project launch files

install(
  DIRECTORY
    models launch urdf meshes scripts urdf_gercek config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/traffic_light.py scripts/game_pad.py scripts/cmd_vel_publisher.py scripts/gta_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
