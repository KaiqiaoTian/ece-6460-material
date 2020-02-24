// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// TF library headers
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>

// Namespace matches ROS package name
namespace ece6460_tf_examples {

  class TfPubExample {
    public:
      TfPubExample(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
        void timerCallback(const ros::TimerEvent& event);
      
      ros::Timer timer_;
      tf2_ros::TransformBroadcaster broadcaster_;

      // Current yaw angle
      double yaw_;

  };

}
