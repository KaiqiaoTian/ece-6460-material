// Header file for the class
#include "TfPubExample.hpp"

// Namespace matches ROS package name
namespace ece6460_tf_examples {

  // Constructor with global and private node handle arguments
  TfPubExample::TfPubExample(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Create a timer to run at 100 Hz and call the 'timerCallback' method every trigger
    timer_ = n.createTimer(ros::Duration(0.01), &TfPubExample::timerCallback, this);

    // Initialize yaw angle to zero
    yaw_ = 0.0;
  }
  
  void TfPubExample::timerCallback(const ros::TimerEvent& event)
  {
    // Update yaw angle
    double yaw_rate = 1.0;
    yaw_ += 0.01 * yaw_rate;

    // Populate transform structure to publish to tf
    geometry_msgs::TransformStamped frame2_to_frame3;
    frame2_to_frame3.header.stamp = event.current_real; // Stamp at current time
    frame2_to_frame3.header.frame_id = "frame2"; // header.frame_id is the parent frame
    frame2_to_frame3.child_frame_id = "frame3"; // child_frame_id is the child frame
    // Fixed translation vector of (2, 1, 1)
    frame2_to_frame3.transform.translation.x = 2.0;
    frame2_to_frame3.transform.translation.y = 1.0;
    frame2_to_frame3.transform.translation.z = 1.0;
    // Populate quaternion corresponding to current yaw angle
    frame2_to_frame3.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_);

    // Publish the transform
    broadcaster_.sendTransform(frame2_to_frame3);
  }

}
