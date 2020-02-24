// Header file for the class
#include "DataGeneratorNode.hpp"

// Namespace matches ROS package name
namespace ece6460_rosbag_examples {

  // Constructor with global and private node handle arguments
  DataGeneratorNode::DataGeneratorNode(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Create a timer to run at 1 Hz and call the 'timerCallback' method every trigger
    timer_ = n.createTimer(ros::Duration(1.0), &DataGeneratorNode::timerCallback, this);

    // Advertise TwistStamped topic that will be recorded to a bag file
    pub_twist_ = n.advertise<geometry_msgs::TwistStamped>("twist", 1);

    dummy_forward_speed_ = 0.0;
  }

  void DataGeneratorNode::timerCallback(const ros::TimerEvent& event)
  {
    // Declare message structure variable
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = event.current_real; // Stamp message at current ROS time
    
    // Cycle dummy data through forward speed field
    dummy_forward_speed_ = fmod(dummy_forward_speed_ + 1.23, 5.55);
    twist_msg.twist.linear.x = dummy_forward_speed_;

    // Constant yaw rate
    twist_msg.twist.angular.z = 0.1;

    // Publish message
    pub_twist_.publish(twist_msg);
  }
}
