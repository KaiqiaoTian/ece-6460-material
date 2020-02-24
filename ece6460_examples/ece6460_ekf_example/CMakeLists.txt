cmake_minimum_required(VERSION 2.8.3)
project(ece6460_ekf_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gps_common
  tf2_ros
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/EkfExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ekf_example
  src/ekf_example_node.cpp
  src/EkfExample.cpp  
)
add_dependencies(ekf_example
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(ekf_example
  ${catkin_LIBRARIES}
)
