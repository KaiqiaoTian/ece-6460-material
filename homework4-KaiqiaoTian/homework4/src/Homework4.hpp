// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS headers
#include <ros/ros.h>

// Message headers
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <homework4/Homework4Config.h>

#include "ObjectEkf.hpp"

// Namespace matches ROS package name
namespace homework4 {

  class Homework4 {
    public:
      Homework4(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void markerTimerCallback(const ros::TimerEvent& event);
      void updateTimerCallback(const ros::TimerEvent& event);
      uint32_t getUniqueId();
      void reconfig(Homework4Config& config, uint32_t level);
      void recvObjects(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg);

      // Methods to predict states and propagate uncertainty 
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);

      ros::Subscriber sub_detected_objects_;
      ros::Publisher pub_object_tracks_;
      ros::Publisher pub_tracking_markers_;
      ros::Timer marker_timer_;
      ros::Timer update_timer_;

      dynamic_reconfigure::Server<Homework4Config> srv_;
      Homework4Config cfg_;

      std::vector<ObjectEkf> object_ekfs_;
      int last_num_markers_;
      static constexpr double DT = 1.0;
  };

}
