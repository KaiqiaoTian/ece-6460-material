// Header file for the class
#include "VectorToQuat.hpp"

// Namespace matches ROS package name
namespace ece6460_vector_to_quat {

  // Constructor with global and private node handle arguments
  VectorToQuat::VectorToQuat(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Create a timer to run at 20 Hz and call the 'timerCallback' method every trigger
    timer_ = n.createTimer(ros::Duration(0.05), &VectorToQuat::timerCallback, this);

    srv_.setCallback(boost::bind(&VectorToQuat::reconfig, this, _1, _2));

    // Advertise topic
    pub_marker_array_ = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
  }

  void VectorToQuat::timerCallback(const ros::TimerEvent& event)
  {
    // Extract components into separate variables for code readability
    double vx = vect.x();
    double vy = vect.y();
    double vz = vect.z();

    // Construct rotation matrix where X axis of transformed frame
    // aligns with the input vector
    tf::Matrix3x3 rot_mat;
    rot_mat[0] = tf::Vector3(vx, vy, vz);
    if (fabs(vz) > 0.9) {
      rot_mat[1] = tf::Vector3(0, vz, -vy);
    } else {
      rot_mat[1] = tf::Vector3(-vy, vx, 0);
    }
    rot_mat[1].normalize();
    rot_mat[2] = rot_mat[0].cross(rot_mat[1]);

    // Convert rotation matrix into equivalent quaternion
    tf::Quaternion q;
    rot_mat.getRotation(q);

    // Populate TF transform
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = event.current_real;
    transform.header.frame_id = "base_frame";
    transform.child_frame_id = "vector_aligned";
    transform.transform.translation.x = 0;
    transform.transform.translation.y = 0;
    transform.transform.translation.z = 0;
    tf::quaternionTFToMsg(q.inverse(), transform.transform.rotation);
    broadcaster_.sendTransform(transform);

    // Create markers to visualize the vectors involved in the computation
    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker m;
    // Common properties
    m.header.frame_id = "base_frame";
    m.header.stamp = event.current_real;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.1;
    m.scale.y = 0.2;
    m.pose.orientation.w = 1;
    m.points.resize(2);
    m.points[0].x = 0.0;
    m.points[0].y = 0.0;
    m.points[0].z = 0.0;

    // Input vector
    m.id = 0;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.points[1].x = vx;
    m.points[1].y = vy;
    m.points[1].z = vz;
    marker_array_msg.markers.push_back(m);

    // Y axis vector
    m.id++;
    m.color.a = 0.2;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.points[1].x = rot_mat[1].x();
    m.points[1].y = rot_mat[1].y();
    m.points[1].z = rot_mat[1].z();
    marker_array_msg.markers.push_back(m);

    // Z axis vector
    m.id++;
    m.color.a = 0.2;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.points[1].x = rot_mat[2].x();
    m.points[1].y = rot_mat[2].y();
    m.points[1].z = rot_mat[2].z();
    marker_array_msg.markers.push_back(m);

    pub_marker_array_.publish(marker_array_msg);
  }

  void VectorToQuat::reconfig(VectorToQuatConfig& config, uint32_t level)
  {
    vect = tf::Vector3(config.x, config.y, config.z);
    vect.normalize();
    config.x = vect.x();
    config.y = vect.y();
    config.z = vect.z();
  }

}
