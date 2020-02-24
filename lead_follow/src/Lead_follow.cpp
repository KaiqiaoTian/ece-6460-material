// Header file for the class
#include "Lead_follow.hpp"


// Namespace matches ROS package name
namespace lead_follow
{

  // Constructor with global and private node handle arguments
  Lead_follow::Lead_follow(ros::NodeHandle& n, ros::NodeHandle& pn):
    listener_(buffer_),
    kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
  {
    sub_detected_objects_ = n.subscribe("fusion_view_objects", 1, &Lead_follow::recvObjects, this);
    sub_joy_node_ = n.subscribe("/joy", 1, &Lead_follow::recvJoy, this);   
    sub_cloud_ = n.subscribe<sensor_msgs::PointCloud2>("fusion/velodyne_points", 10, &Lead_follow::recvCloud, this);
    
    pub_filtered = n.advertise<sensor_msgs::PointCloud2>("filtered_pass", 1);
    pub_no_ground_cloud = n.advertise<sensor_msgs::PointCloud2>("no_ground_cloud", 1);
    pub_merged_cluster_cloud_ = n.advertise<sensor_msgs::PointCloud2>("merged_cluster_cloud", 1);
    pub_bboxes_ = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("fusion_view_objects", 1);
    pub_normals_ = n.advertise<geometry_msgs::PoseArray>("normals", 1);
    pub_pacifica_path_ = n.advertise<nav_msgs::Path>("pacifica_path", 10);
    pub_fusion_path_ = n.advertise<nav_msgs::Path>("fusion_path", 10);
    pub_fusion_vel_ = n.advertise<geometry_msgs::Twist>("fusion/cmd_vel", 1);
    pub_fusion_twist_ = n.advertise<dataspeed_ulc_msgs::UlcCmd>("fusion/ulc_cmd", 1);
    pub_pacifica_twist_ = n.advertise<geometry_msgs::Twist>("pacifica/cmd_vel", 1);
    pub_object_tracks_ = n.advertise<autoware_msgs::DetectedObjectArray>("lead_follow/object_tracks", 1);
    pub_tracking_markers_ = n.advertise<visualization_msgs::MarkerArray>("lead_follow/tracked_object_markers", 1);
    marker_timer_ = n.createTimer(ros::Duration(0.05), &Lead_follow::markerTimerCallback, this);
    update_timer_ = n.createTimer(ros::Duration(0.02), &Lead_follow::updateTimerCallback, this);
    follow_vehicle_control_timer_ = n.createTimer(ros::Duration(0.05), &Lead_follow::controlTimerCallback, this);

    srv_.setCallback(boost::bind(&Lead_follow::reconfig, this, _1, _2));
  }

  void Lead_follow::recvJoy(const sensor_msgs::JoyConstPtr& joy)
  {
    sensor_msgs::Joy read_joy;
    geometry_msgs::Twist twist_msg;
    int start_button;
    start_button = joy -> buttons[5];
    if (start_button < 0.9){
      twist_msg.linear.x = (joy -> axes[1])*5; //velocity gain  origin value 1
      twist_msg.angular.z = (joy -> axes[3])*0.5; //angular gain
      pub_pacifica_twist_.publish(twist_msg);
    }else
    {
      twist_msg.linear.x = 0; //set 0
      twist_msg.angular.z = 0; //set 0
      pub_pacifica_twist_.publish(twist_msg);
    }
    
  }

  void Lead_follow::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    tf::Transform transform;
    
    try {
      geometry_msgs::TransformStamped tr = buffer_.lookupTransform("fusion/base_footprint", msg->header.frame_id, ros::Time(0));
      tf::transformMsgToTF(tr.transform, transform);
    } catch (tf2::TransformException& ex) {
      ROS_WARN_STREAM_THROTTLE(1.0, ex.what());
      return;
    }

    // Apply coordinate frame transformation
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("fusion/base_footprint", transform, *msg, transformed_msg);

    // Copy into PCL cloud for processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // TODO: Copy transformed_msg into input_cloud
    pcl::fromROSMsg(transformed_msg, *input_cloud);

    // Run the processing pipeline
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    passthroughFilter(input_cloud, filtered_cloud);
    voxelFilter(filtered_cloud, filtered_cloud);
    sensor_msgs::PointCloud2 output_filtered;
    pcl::toROSMsg(*filtered_cloud, output_filtered);
    output_filtered.header = pcl_conversions::fromPCL(filtered_cloud->header);

    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    normalsFilter(filtered_cloud, no_ground_cloud);
    sensor_msgs::PointCloud2 output_no_ground_cloud;
    pcl::toROSMsg(*no_ground_cloud, output_no_ground_cloud);
    output_no_ground_cloud.header = pcl_conversions::fromPCL(filtered_cloud->header);



    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    euclideanClustering(no_ground_cloud, cluster_clouds);
    generateBoundingBoxes(cluster_clouds);
    bboxes_.header = pcl_conversions::fromPCL(no_ground_cloud->header);

   
    //pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cluster_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    mergeClusters(cluster_clouds);
    //pcl::toROSMsg(*merged_cluster_cloud_, merged_cluster_cloud_1);
    merged_cluster_cloud_.header = pcl_conversions::fromPCL(no_ground_cloud->header);

    //filtered_cloud.header = pcl_conversions::fromPCL(filtered_cloud->header);

    pub_merged_cluster_cloud_.publish(merged_cluster_cloud_);
    pub_filtered.publish(output_filtered);
    pub_no_ground_cloud.publish(output_no_ground_cloud);
    pub_bboxes_.publish(bboxes_);
    pub_normals_.publish(normals_);
  }


 // void Lead_follow::reconfig(Lead_followConfig& config, uint32_t level)
 // {
 //   cfg_ = config;
 // }


  void Lead_follow::passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // TODO: Implement passthrough filter here. Put final output in the 'cloud_out' argument
    pcl::IndicesPtr roi_indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Give passthrough filter the pointer to the cloud we want to filter
    pass.setInputCloud (cloud_in);

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (cfg_.x_min, cfg_.x_max);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Y range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (cfg_.y_min, cfg_.y_max);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Z range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (cfg_.z_min, cfg_.z_max);
    pass.filter (*cloud_out);


  }

  void Lead_follow::voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // TODO: Implement voxel downsampling filter here. Put final output in the 'cloud_out' argument
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(cloud_in);
    downsample.setLeafSize(cfg_.voxel_size, cfg_.voxel_size ,cfg_.voxel_size);
    downsample.filter(*cloud_out);

  }

  void Lead_follow::normalsFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Compute normal vectors for the incoming point cloud
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    kd_tree_->setInputCloud(cloud_in);
    normal_estimator.setSearchMethod(kd_tree_);
    normal_estimator.setInputCloud(cloud_in);
    normal_estimator.setKSearch(cfg_.num_normal_neighbors);
    normal_estimator.compute(*cloud_normals);

    // Filter out near-vertical normals
    pcl::PointIndices non_vertical_normals;
    //pcl::Normal normal_reference(0,0,1);
    //pcl::Normal normal_data;
    double angle_up;
    double angle_down;
    for (int i = 0; i < cloud_normals->points.size(); i++) {
      // TODO: Compute the angle from vertical for the current normal vector
      //       and if it is greater than 30 degrees from vertical, add the index
      //       to the 'non_vertical_normals' array
    double z_up = cloud_normals->points[i].normal_z;
    double z_down = -cloud_normals->points[i].normal_z;
      angle_up = acos(z_up)*180. / M_PI;
      angle_down = acos(z_down)*180. / M_PI;
      if (angle_up > cfg_.ground_normal_angle && angle_down > cfg_.ground_normal_angle){
          non_vertical_normals.indices.push_back(i);
      }

    }
    pcl::copyPointCloud(*cloud_in, non_vertical_normals, *cloud_out);

    // Populate PoseArray message to visualize normals
    normals_.header = pcl_conversions::fromPCL(cloud_in->header);
    normals_.poses.clear();
    for (int i = 0; i < non_vertical_normals.indices.size(); i++) {
      geometry_msgs::Pose p;
      p.position.x = cloud_in->points[non_vertical_normals.indices[i]].x;
      p.position.y = cloud_in->points[non_vertical_normals.indices[i]].y;
      p.position.z = cloud_in->points[non_vertical_normals.indices[i]].z;

      double nx = cloud_normals->points[non_vertical_normals.indices[i]].normal_x;
      double ny = cloud_normals->points[non_vertical_normals.indices[i]].normal_y;
      double nz = cloud_normals->points[non_vertical_normals.indices[i]].normal_z;

      tf2::Quaternion q;
      tf2::Matrix3x3 rot_mat;
      if (fabs(nz) < 0.01) {
        rot_mat[0] = tf2::Vector3(nx, ny, nz);
        rot_mat[1] = tf2::Vector3(0, -nz, ny);
        rot_mat[1].normalize();
        rot_mat[2] = rot_mat[0].cross(rot_mat[1]);
      } else {
        rot_mat[0] = tf2::Vector3(nx, ny, nz);
        rot_mat[1] = tf2::Vector3(-ny, nx, 0);
        rot_mat[1].normalize();
        rot_mat[2] = rot_mat[0].cross(rot_mat[1]);
      }
      rot_mat.getRotation(q);
      tf2::convert(q.inverse(), p.orientation);
      normals_.poses.push_back(p);
    }
  }

  void Lead_follow::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    // TODO: Implement Euclidean clustering here, dumping the array of separated clouds into the 'cluster_clouds' argument
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cfg_.cluster_tol);
    ec.setMinClusterSize(cfg_.min_cluster_size);
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    kd_tree_->setInputCloud(cloud_in);
    ec.setSearchMethod(kd_tree_);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);

    // Use indices arrays to separate point cloud into individual clouds for each cluster
   // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    for (auto indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud_in, indices, *cluster);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster_clouds.push_back(cluster);
    }
  
  
  }

  void Lead_follow::mergeClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    // TODO: Loop through the 'cluster_clouds' argument and merge all the points into a single PCL
    //       point cloud. Then copy the merged cloud into the 'merged_cluster_cloud_' message

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& cluster : cluster_clouds) {
      merged_cloud->points.insert(merged_cloud->points.begin(), cluster->points.begin(), cluster->points.end());
    }
    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;

    pcl::toROSMsg(*merged_cloud, merged_cluster_cloud_);

  }

  void Lead_follow::generateBoundingBoxes(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    pcl::PointXYZ min_point, max_point;
    bboxes_.boxes.clear();
    int bbox_id = 0;
    for (auto& cluster : cluster_clouds) {
      pcl::getMinMax3D(*cluster, min_point, max_point);
      jsk_recognition_msgs::BoundingBox box;
      box.header = bboxes_.header;
      // TODO: Fill in the rest of the 'box' variable
      //         - Increment the 'label' field for each cluster
      //         - Populate 'pose.position' with the midpoint between min_point and max_point
      //         - Populate 'pose.orientation' with an identity quaternion
      //         - Populate 'dimensions' with data from min_point and max_point
      
      box.dimensions.x = max_point.x - min_point.x;
      box.dimensions.y = max_point.y - min_point.y;
      box.dimensions.z = max_point.z - min_point.z;
     // if(box.dimensions.x> 6.0) continue;
      //if(box.dimensions.y> 6.0) continue;
      box.label=bbox_id++;
      box.pose.position.x = 0.5 * (max_point.x + min_point.x);
      box.pose.position.y = 0.5 * (max_point.y + min_point.y);
      box.pose.position.z = 0.5 * (max_point.z + min_point.z);
      box.pose.orientation.w = 1.0;
      bboxes_.boxes.push_back(box);
    }
  
  }





  void Lead_follow::updateTimerCallback(const ros::TimerEvent& event)
  {
    // Delete stale objects that have not been observed for a while
    std::vector<size_t> stale_objects;
    for (size_t i = 0; i < object_ekfs_.size(); i++) {
      object_ekfs_[i].updateFilterPredict(event.current_real);
      if (object_ekfs_[i].isStale()) {
        stale_objects.push_back(i);
      }
    }
    for (int i = (int)stale_objects.size() - 1; i >= 0; i--) {
      object_ekfs_.erase(object_ekfs_.begin() + stale_objects[i]);
    }

    // Generate detected object outputs
    autoware_msgs::DetectedObjectArray object_track_msg;
    object_track_msg.header.stamp = event.current_real;
    for (size_t i = 0; i < object_ekfs_.size(); i++) {
      if (object_ekfs_[i].getAge() < cfg_.min_age) {
        continue;
      }
      object_track_msg.objects.push_back(object_ekfs_[i].getEstimate());
    }
    pub_object_tracks_.publish(object_track_msg);
  }

  void Lead_follow::recvObjects(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg)
  {
    // Vector to hold the EKF indices that have already been matched to an incoming object measurement
    std::vector<int> matched_object_indices;

    // Vector to hold array indices of objects to create new EKF instances from
    std::vector<int> new_object_indices;

    // Loop through all incoming object measurements and associate them with an existing EKF, or create
    // a new EKF instance and initialize its state to the measured values
    for (size_t i = 0; i < msg->boxes.size(); i++) {
      const jsk_recognition_msgs::BoundingBox& object = msg->boxes[i];
      if (fabs(object.pose.position.y) > cfg_.y_window) {
        continue;
      }
      if (object.dimensions.z < cfg_.min_size_z) {
        continue;
      }
      
      // Loop through each existing EKF instance and find the one closest to the current object measurement
      double min_dist2 = INFINITY;
      int associated_track_idx = -1;
      for (size_t j = 0; j < object_ekfs_.size(); j++) {
        // If the current EKF instance has already been associated with a measurement, skip it and try the next one
        if (std::find(matched_object_indices.begin(), matched_object_indices.end(), j) != matched_object_indices.end()) {
          continue;
        }

        // Compute the distance between the EKF estimate of the position and the position of the measurement
        geometry_msgs::Point est_pos = object_ekfs_[j].getEstimate().pose.position;
        tf::Vector3 est_pos_vect(est_pos.x, est_pos.y, 0.0);
        tf::Vector3 meas_pos_vect(object.pose.position.x, object.pose.position.y, 0.0);
        double d2 = (meas_pos_vect - est_pos_vect).length2();

        // If the distance is the smallest so far, mark this EKF instance as the association candidate
        if (d2 < min_dist2) {
          min_dist2 = d2;
          associated_track_idx = (int)j;
        }
      }

      if ((associated_track_idx < 0) || (min_dist2 > (cfg_.max_match_dist * cfg_.max_match_dist))) {
        // If no EKF instances exist yet, or the closest match is too far away, mark this
        // object to create a new EKF instance to track it
        new_object_indices.push_back(i);
      } else {
        // Object measurement successfully associated with an existing EKF instance...
        // Update that EKF and mark it as already associated so another object in
        //     the same measurement array doesn't also get associated to it
        object_ekfs_[associated_track_idx].updateFilterMeasurement(object);
        matched_object_indices.push_back(associated_track_idx);
      }
    }

    // After trying to associate all incoming object measurements to existing EKF instances,
    // create new EKF instances to track the inputs that weren't associated with existing ones
    for (auto new_object_idx : new_object_indices) {
      object_ekfs_.push_back(ObjectEkf(msg->boxes[new_object_idx].pose.position.x, 0.0, 
                                       msg->boxes[new_object_idx].pose.position.y, 0.0,
                                       msg->boxes[new_object_idx].header.stamp, msg->header.frame_id));
      object_ekfs_.back().setQ(cfg_.q_pos, cfg_.q_vel);
      object_ekfs_.back().setR(cfg_.r_pos);
    }
  }

  void Lead_follow::markerTimerCallback(const ros::TimerEvent& event)
  {
    // Populate marker array message with boxes and arrows showing the position
    // and relative velocity output from each EKF instance
    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::ADD;
    m.id = 0;
    m.color.a = 1.0;
    m.points.resize(2);
    for (auto& ekf : object_ekfs_) {
      autoware_msgs::DetectedObject estimate = ekf.getEstimate();
      m.type = visualization_msgs::Marker::CUBE;
      m.header.frame_id = estimate.header.frame_id;
      m.pose = estimate.pose;
      m.scale = estimate.dimensions;
      m.color.a = 0.5;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 1.0;
      marker_array_msg.markers.push_back(m);
      m.id++;

      m.type = visualization_msgs::Marker::ARROW;
      if (estimate.velocity.linear.x < 0.1 && estimate.velocity.linear.y < 0.1){
      m.points[1].x = 0.01 * estimate.velocity.linear.x;
      m.points[1].y = 0.01 * estimate.velocity.linear.y;
      }else
      {
      m.points[1].x = DT * 10 * estimate.velocity.linear.x;
      m.points[1].y = DT * 10 * estimate.velocity.linear.y;        
      }
      m.color.a = 1.0;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.scale.x = 0.5;
      m.scale.y = 1.0;
      m.scale.z = 0.0;
      marker_array_msg.markers.push_back(m);
      m.id++;


    }

    // Delete all markers when size changes to avoid leaving ghosts in Rviz
    if (last_num_markers_ != marker_array_msg.markers.size()) {
      visualization_msgs::MarkerArray clear_markers;
      clear_markers.markers.resize(1);
      clear_markers.markers[0].action = visualization_msgs::Marker::DELETEALL;
      pub_tracking_markers_.publish(clear_markers);
    }
    last_num_markers_ = marker_array_msg.markers.size();

    pub_tracking_markers_.publish(marker_array_msg);
  }


  /// follow vehicle control function
  void Lead_follow::controlTimerCallback(const ros::TimerEvent& event)
  {

    for (auto& ekf : object_ekfs_) {
      autoware_msgs::DetectedObject estimate = ekf.getEstimate();
      // get leader vehicle x.y position
      double leader_pose_x = estimate.pose.position.x;
      double leader_pose_y = estimate.pose.position.y;
      double leader_angle_x = 10 * estimate.velocity.linear.x;
      double leader_angle_y = 10 * estimate.velocity.linear.y;
      // ROS_INFO("x and y : (%f, %f)", leader_pose_x, leader_pose_y);


      double l = sqrt(leader_angle_x*leader_angle_x + leader_angle_y*leader_angle_y);
      double r = sqrt(leader_pose_x*leader_pose_x + leader_pose_y*leader_pose_y);
      double follow_angle = atan(leader_pose_x / leader_pose_y);
      double leader_angle = acos((leader_pose_x*leader_angle_x +leader_pose_y*leader_angle_y )/(r*l));
      double k_1 = 1;
      double k_2 = 3;
      double k_smooth = (-1/r)*(k_2*(follow_angle-atan(-k_1*leader_angle)+(1+(k_1/(1+(k_1*k_1*leader_angle*leader_angle))))*sin(follow_angle)));
      double k_simple = (2*leader_pose_y/(leader_pose_x*leader_pose_x + leader_pose_y*leader_pose_y));
      //ROS_INFO("follow curvature and r  : (%f,%f)", k_simple,r);
      
      //////publish ulc_cmd to fusion, using curvature///////////////////////
      dataspeed_ulc_msgs::UlcCmd fusion_ulc_msg;
      geometry_msgs::Twist fusion_twist_msg;
      geometry_msgs::PoseStamped new_path_point;
      geometry_msgs::PoseStamped new_path_point_pacifica;
     
      tf::StampedTransform transform_path;
      tf::TransformListener listen_fusion_footprint;
      
      
     // listen_fusion_footprint.lookupTransform("/world", "/fusion/base_footpri````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````nt", ros::Time(0), transform_path);   
         ///tf frame from second to first
       
    tf::Transform transform_fusion;
    tf::Transform transform_pacifaca;
    
    try {
      geometry_msgs::TransformStamped tr_fusion = buffer_.lookupTransform( "world","fusion/base_footprint", ros::Time(0));
      tf::transformMsgToTF(tr_fusion.transform, transform_fusion);

      geometry_msgs::TransformStamped tr_pacifica = buffer_.lookupTransform( "world","pacifica/base_footprint", ros::Time(0));
      tf::transformMsgToTF(tr_pacifica.transform, transform_pacifaca);

    } catch (tf2::TransformException& ex) {
      ROS_WARN_STREAM_THROTTLE(1.0, ex.what());
      ros::Duration(1.0).sleep();
      return;
    } 
      
      new_path_point.pose.position.x = transform_fusion.getOrigin().getX();
      new_path_point.pose.position.y = transform_fusion.getOrigin().getY();
      new_path_point.header.frame_id = "world";
      fusion_path.header.stamp = event.current_real;
      fusion_path.header.frame_id = "world";

     
      fusion_path.poses.push_back(new_path_point);
      pub_fusion_path_.publish(fusion_path);


      new_path_point_pacifica.pose.position.x = transform_pacifaca.getOrigin().getX();
      new_path_point_pacifica.pose.position.y = transform_pacifaca.getOrigin().getY();
      new_path_point_pacifica.header.frame_id = "world";
      pacifica_path.header.stamp = event.current_real;
      pacifica_path.header.frame_id = "world";

      pacifica_path.poses.push_back(new_path_point_pacifica);
      pub_pacifica_path_.publish(pacifica_path);
     // ROS_INFO("pacifica footprint x,y : (%f,%f)", new_path_point_pacifica.pose.position.x,new_path_point_pacifica.pose.position.y);



      double follow_v = 5;
      fusion_ulc_msg.clear = false;
      fusion_ulc_msg.enable_pedals = true;
      fusion_ulc_msg.enable_steering = true;
      fusion_ulc_msg.steering_mode = 1;

      if (r > 6){
        fusion_ulc_msg.linear_velocity = follow_v;
        fusion_ulc_msg.yaw_command = k_simple;
        pub_fusion_twist_.publish(fusion_ulc_msg);

      }else
      {
        fusion_ulc_msg.linear_velocity = 0;
        fusion_ulc_msg.yaw_command = k_simple;
        pub_fusion_twist_.publish(fusion_ulc_msg);

      }
      /////////////////////////////////////////////////////////
      /////////publish cmd msg to fusion/////////
    
  

    }
  }



  void Lead_follow::reconfig(Lead_followConfig& config, uint32_t level)
  {
    cfg_ = config;

    // Update Q and R matrices in each EKF instance
    for (size_t i = 0; i < object_ekfs_.size(); i++) {
      object_ekfs_[i].setQ(cfg_.q_pos, cfg_.q_vel);
      object_ekfs_[i].setR(cfg_.r_pos);
    }
  }

}
