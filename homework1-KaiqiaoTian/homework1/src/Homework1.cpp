// Header file for the class
#include "Homework1.hpp"

// Namespace matches ROS package name
namespace homework1 
{  
  // Constructor with global and private node handle arguments
  Homework1::Homework1(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    sub_twist_ = n.subscribe("/vehicle/twist", 1, &Homework1::recvTwist, this);
    pub_speed_data_ = n.advertise<std_msgs::Float64>("/vehicle_speed", 1);
  }

  void Homework1::recvTwist(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    geometry_msgs::TwistStamped read_msg;
    std_msgs::Float64 pub_msg;
    pub_msg.data = msg -> twist.linear.x;
    pub_speed_data_.publish(pub_msg);
  }
}
