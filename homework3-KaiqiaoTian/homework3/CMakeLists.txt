cmake_minimum_required(VERSION 2.8.3)
project(homework3)

### [Mandatory] List all ROS packages this package is dependent on.
#               This list should match the packages with <depend> tags in package.xml
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  tf
  dynamic_reconfigure

)

### [Recommended] Use C++ 11 compiler
add_compile_options(-std=c++11)

### Generate headers for dynamic_reconfigure server
#generate_dynamic_reconfigure_options()

generate_dynamic_reconfigure_options(
  cfg/homework3.cfg
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
add_executable(homework3
 src/homework3_node.cpp
 src/Homework3.cpp
)
add_dependencies(homework3
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(homework3
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
