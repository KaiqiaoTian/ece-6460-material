// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

// Namespace matches ROS package name
namespace homework1 {

  class Homework1 {
    public:
      Homework1(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      // Node-specific stuff here

      void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);

      ros::Subscriber sub_twist_;
      ros::Publisher pub_speed_data_;
  };

}
