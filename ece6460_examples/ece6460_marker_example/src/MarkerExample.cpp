// Header file for the class
#include "MarkerExample.hpp"
#include <tf/tf.h>

// Namespace matches ROS package name
namespace ece6460_marker_example {

  // Constructor with global and private node handle arguments
  MarkerExample::MarkerExample(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Create a timer to run at 20 Hz and call the 'timerCallback' method every trigger
    timer_ = n.createTimer(ros::Duration(0.05), &MarkerExample::timerCallback, this);

    // Advertise topic
    pub_marker_array_ = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

    // Set up constant values for marker messages:
    // Cube marker
    initCubeMarker();

    // Arrow marker
    initArrowMarker();

    // Text marker
    initTextMarker();
  }

  void MarkerExample::initCubeMarker()
  {
    cube_marker_msg_.header.frame_id = "frame1";
    cube_marker_msg_.id = 0;
    cube_marker_msg_.action = visualization_msgs::Marker::ADD;
    cube_marker_msg_.type = visualization_msgs::Marker::CUBE;
    
    cube_marker_msg_.pose.position.x = 3.0;
    cube_marker_msg_.pose.position.y = 3.0;
    cube_marker_msg_.pose.position.z = 1.0;
    cube_marker_msg_.pose.orientation.w = 1.0;
    
    cube_marker_msg_.scale.x = 1.0; // Cube length
    cube_marker_msg_.scale.y = 1.0; // Cube width
    cube_marker_msg_.scale.z = 1.0; // Cube height

    cube_marker_msg_.color.a = 0.75;
    cube_marker_msg_.color.r = 1.0;
    cube_marker_msg_.color.g = 1.0;
    cube_marker_msg_.color.b = 0.0;
  }

  void MarkerExample::initArrowMarker()
  {
    arrow_marker_msg_.header.frame_id = "frame2";
    arrow_marker_msg_.id = 1;
    arrow_marker_msg_.action = visualization_msgs::Marker::ADD;
    arrow_marker_msg_.type = visualization_msgs::Marker::ARROW;
    
    arrow_marker_msg_.pose.position.x = 0.0;
    arrow_marker_msg_.pose.position.y = 0.0;
    arrow_marker_msg_.pose.position.z = 0.0;
    arrow_marker_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 4);
    
    arrow_marker_msg_.scale.x = 2.0; // Arrow length
    arrow_marker_msg_.scale.y = 0.1; // Arrow width
    arrow_marker_msg_.scale.z = 0.1; // Arrow height
    
    arrow_marker_msg_.color.a = 1.0;
    arrow_marker_msg_.color.r = 1.0;
    arrow_marker_msg_.color.g = 0.0;
    arrow_marker_msg_.color.b = 1.0;
  }

  void MarkerExample::initTextMarker()
  {
    text_marker_msg_.header.frame_id = "frame1";
    text_marker_msg_.id = 2;
    text_marker_msg_.action = visualization_msgs::Marker::ADD;
    text_marker_msg_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    text_marker_msg_.pose.position.x = 0.0;
    text_marker_msg_.pose.position.y = 0.0;
    text_marker_msg_.pose.position.z = 0.0;
    text_marker_msg_.pose.orientation.w = 1.0;

    text_marker_msg_.scale.z = 1.0;
    text_marker_msg_.text = "insert_text_here";

    text_marker_msg_.color.a = 1.0;
    text_marker_msg_.color.r = 0.0;
    text_marker_msg_.color.g = 1.0;
    text_marker_msg_.color.b = 0.0;
  }

  void MarkerExample::timerCallback(const ros::TimerEvent& event)
  {
    // Update marker time stamps
    cube_marker_msg_.header.stamp = event.current_real;
    arrow_marker_msg_.header.stamp = event.current_real;
    text_marker_msg_.header.stamp = event.current_real;
  
    // Combine markers into a MarkerArray message and publish it
    visualization_msgs::MarkerArray marker_array_msg;
    marker_array_msg.markers.push_back(cube_marker_msg_);
    marker_array_msg.markers.push_back(arrow_marker_msg_);
    marker_array_msg.markers.push_back(text_marker_msg_);
    pub_marker_array_.publish(marker_array_msg);
  }

}
