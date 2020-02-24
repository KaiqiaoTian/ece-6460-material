// ROS and node class header file
#include <ros/ros.h>
#include "MarkerExample.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "marker_example_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_marker_example::MarkerExample node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
