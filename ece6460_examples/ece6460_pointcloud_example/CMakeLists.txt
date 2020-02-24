cmake_minimum_required(VERSION 2.8.3)
project(ece6460_pointcloud_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  dynamic_reconfigure
  pcl_ros
)

add_compile_options(-std=c++11)

generate_dynamic_reconfigure_options(
  cfg/PointCloudExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_example
  src/pointcloud_example_node.cpp
  src/PointCloudExample.cpp
)
add_dependencies(pointcloud_example
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(pointcloud_example
  ${catkin_LIBRARIES}
)

add_executable(cluster_example
  src/cluster_example_node.cpp
  src/ClusterExample.cpp
)
add_dependencies(cluster_example
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(cluster_example
  ${catkin_LIBRARIES}
)
