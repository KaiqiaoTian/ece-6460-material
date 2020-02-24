// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <homework2/Homework2Config.h>

//ROS message headers
#include <gps_common/GPSFix.h>
#include <autoware_msgs/Lane.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>
#include <gps_common/conversions.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// Namespace matches ROS package name
namespace homework2 {

  class Homework2 {
    public:
      Homework2(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void reconfig(Homework2Config& config, uint32_t level);
      void recvWaypoints(const autoware_msgs::LaneConstPtr& msg);
      void recvFix(const gps_common::GPSFixConstPtr& msg);

      ros::Subscriber sub_fix_;
      ros::Subscriber sub_waypoints_;
      ros::Publisher pub_speed_data_;

      dynamic_reconfigure::Server<Homework2Config> srv_;
      Homework2Config cfg_;

      std::vector<autoware_msgs::Waypoint> utm_waypoints;
      



  };

}
