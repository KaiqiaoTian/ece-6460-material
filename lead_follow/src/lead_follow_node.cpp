// ROS and node class header file
#include <ros/ros.h>
#include "Lead_follow.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "lead_follow");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  lead_follow::Lead_follow node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
