// ROS and node class header file
#include <ros/ros.h>
#include "Homework3.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "homework3");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  homework3::Homework3 node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
