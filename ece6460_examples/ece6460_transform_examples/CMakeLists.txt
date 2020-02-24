cmake_minimum_required(VERSION 2.8.3)
project(ece6460_transform_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_example1
  src/tf_example1.cpp
)
add_dependencies(transform_example1
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(transform_example1
  ${catkin_LIBRARIES}
)

add_executable(transform_example2
  src/tf_example2.cpp
)
add_dependencies(transform_example2
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(transform_example2
  ${catkin_LIBRARIES}
)

add_executable(gps_transform_example
  src/gps_transform_example.cpp
)
add_dependencies(gps_transform_example
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gps_transform_example
  ${catkin_LIBRARIES}
)
