#include <ros/ros.h>
#include "LaneDetection.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_detection");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  ece6460_mono_example::LaneDetection node(n, pn);
  
  ros::spin();
}