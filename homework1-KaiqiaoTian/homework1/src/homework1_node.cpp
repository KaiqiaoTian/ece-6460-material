// ROS and node class header file
#include <ros/ros.h>
#include "Homework1.hpp"


int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "homework1");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  homework1::Homework1 node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
