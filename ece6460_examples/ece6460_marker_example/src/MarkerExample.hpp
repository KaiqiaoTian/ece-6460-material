// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// ROS message headers
#include <visualization_msgs/MarkerArray.h>

// Namespace matches ROS package name
namespace ece6460_marker_example {

  class MarkerExample {
    public:
      MarkerExample(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void timerCallback(const ros::TimerEvent& event);

      void initCubeMarker();
      void initArrowMarker();
      void initTextMarker();

      ros::Timer timer_;
      ros::Publisher pub_marker_array_;

      visualization_msgs::Marker cube_marker_msg_;
      visualization_msgs::Marker arrow_marker_msg_;
      visualization_msgs::Marker text_marker_msg_;
  };

}
