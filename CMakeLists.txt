cmake_minimum_required(VERSION 3.8)
project(oomwoo_clean_and_map)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/coverage_planner_node.py
    scripts/save_and_analyze_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
