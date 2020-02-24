cmake_minimum_required(VERSION 2.8.3)
project(lead_follow)

### [Mandatory] List all ROS packages this package is dependent on.
#               This list should match the packages with <depend> tags in package.xml
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  visualization_msgs
  pcl_ros
  tf
)

### [Recommended] Use C++ 11 compiler
add_compile_options(-std=c++11)

### Generate headers for dynamic_reconfigure server
generate_dynamic_reconfigure_options(
  cfg/Lead_follow.cfg
)

### [Mandatory] Identify this CMake project as a catkin package
catkin_package()

#### [Mandatory] Include exported include directories from other packages
#                this package is dependent on.
include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Instruct catkin_make to compile a node executable program
#   First argument is the name of the resulting executable
#    After the executable name, list all .cpp files required
#        to compile the node. Keep in mind that the list of 
#        source files must consist of relative paths to the
#        files from the root of the package folder
add_executable(lead_follow
 src/lead_follow_node.cpp
 src/Lead_follow.cpp
 src/ObjectEkf.cpp
)
add_dependencies(lead_follow
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(lead_follow
  ${catkin_LIBRARIES}
)

