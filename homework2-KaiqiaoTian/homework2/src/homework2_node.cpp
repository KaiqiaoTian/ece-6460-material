// ROS and node class header file
#include <ros/ros.h>
#include "Homework2.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "homework2");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  homework2::Homework2 node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
