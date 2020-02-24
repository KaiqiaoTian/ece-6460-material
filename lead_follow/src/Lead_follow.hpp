// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once


// ROS headers
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>
#include <nav_msgs/Path.h>

// Message headers
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

// PCL processing headers
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <lead_follow/Lead_followConfig.h>

//math
#include <math.h>
#include "ObjectEkf.hpp"

// Namespace matches ROS package name
namespace lead_follow {

  class Lead_follow {
    public:
      Lead_follow(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void markerTimerCallback(const ros::TimerEvent& event);
      void updateTimerCallback(const ros::TimerEvent& event);
      void controlTimerCallback(const ros::TimerEvent& event);
      uint32_t getUniqueId();
      void reconfig(Lead_followConfig& config, uint32_t level);
      void recvObjects(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg);
      void recvJoy(const sensor_msgs::JoyConstPtr& joy);

      void recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

      // Pipeline stage functions
      void passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

      void voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

      void normalsFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

      void euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                               std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds);

      void mergeClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds);

      void generateBoundingBoxes(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds);


      // Methods to predict states and propagate uncertainty 
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);

      ros::Subscriber sub_detected_objects_;
      ros::Subscriber sub_joy_node_;

      ros::Publisher pub_pacifica_path_;
      ros::Publisher pub_fusion_path_;
      ros::Publisher pub_fusion_vel_;   //cmd_vel
      ros::Publisher pub_fusion_twist_; //ulc_vel
      ros::Publisher pub_pacifica_twist_;
      ros::Publisher pub_object_tracks_;
      ros::Publisher pub_tracking_markers_;
      
       // Subscribers and publishers
      ros::Subscriber sub_cloud_;
      ros::Publisher pub_merged_cluster_cloud_;
      ros::Publisher pub_bboxes_;
      ros::Publisher pub_normals_;
      ros::Publisher pub_filtered;
      ros::Publisher pub_no_ground_cloud;

      // For looking up transforms
      //tf2_ros::TransformListener path_; ///tf
      tf2_ros::Buffer buffer_;
      tf2_ros::TransformListener listener_;
     // tf2_ros::Buffer buffer_fusion_;
      //tf2_ros::TransformListener listener_fusion_;


      // Output messages
      sensor_msgs::PointCloud2 filtered;
      //sensor_msgs::PointCloud2 no_ground_cloud;
      sensor_msgs::PointCloud2 merged_cluster_cloud_;
     // sensor_msgs::PointCloud2 merged_cluster_cloud_1;
      geometry_msgs::PoseArray normals_;
      jsk_recognition_msgs::BoundingBoxArray bboxes_;

      //path message
      nav_msgs::Path fusion_path;
      nav_msgs::Path pacifica_path;


      // KD search tree object for use by PCL functions
      pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;


      ros::Timer marker_timer_;
      ros::Timer update_timer_;
      ros::Timer follow_vehicle_control_timer_;

      dynamic_reconfigure::Server<Lead_followConfig> srv_;
      Lead_followConfig cfg_;

      std::vector<ObjectEkf> object_ekfs_;
      int last_num_markers_;
      static constexpr double DT = 1.0;
  };

}
