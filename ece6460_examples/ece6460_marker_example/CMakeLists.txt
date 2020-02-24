cmake_minimum_required(VERSION 2.8.3)
project(ece6460_marker_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_example
  src/marker_example_node.cpp
  src/MarkerExample.cpp
)
add_dependencies(marker_example
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(marker_example
  ${catkin_LIBRARIES}
)

