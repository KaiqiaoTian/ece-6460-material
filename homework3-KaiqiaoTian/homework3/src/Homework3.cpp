// Header file for the class
#include "Homework3.hpp"
#include <tf/tf.h>


// Namespace matches ROS package name
namespace homework3 
{  
  // Constructor with global and private node handle arguments
  Homework3::Homework3(ros::NodeHandle& n, ros::NodeHandle& pn) :
    listener_(buffer_),
    kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
  {
    sub_cloud_ = n.subscribe<sensor_msgs::PointCloud2>("ouster/points_raw", 10, &Homework3::recvCloud, this);
    pub_filtered = n.advertise<sensor_msgs::PointCloud2>("filtered_pass", 1);
    pub_no_ground_cloud = n.advertise<sensor_msgs::PointCloud2>("no_ground_cloud", 1);
    pub_merged_cluster_cloud_ = n.advertise<sensor_msgs::PointCloud2>("merged_cluster_cloud", 1);
    pub_bboxes_ = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("objects", 1);
    pub_normals_ = n.advertise<geometry_msgs::PoseArray>("normals", 1);
  }

  void Homework3::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    tf::Transform transform;
    
    try {
      geometry_msgs::TransformStamped tr = buffer_.lookupTransform("base_footprint", msg->header.frame_id, ros::Time(0));
      tf::transformMsgToTF(tr.transform, transform);
    } catch (tf2::TransformException& ex) {
      ROS_WARN_STREAM_THROTTLE(1.0, ex.what());
      return;
    }

    // Apply coordinate frame transformation
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("base_footprint", transform, *msg, transformed_msg);

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


 // void Homework3::reconfig(homework3Config& config, uint32_t level)
 // {
 //   cfg_ = config;
 // }


  void Homework3::passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // TODO: Implement passthrough filter here. Put final output in the 'cloud_out' argument
    pcl::IndicesPtr roi_indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Give passthrough filter the pointer to the cloud we want to filter
    pass.setInputCloud (cloud_in);

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 100.0);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Y range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-10.0, 10.0);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Z range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-2.0, 3.0);
    pass.filter (*cloud_out);


  }

  void Homework3::voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // TODO: Implement voxel downsampling filter here. Put final output in the 'cloud_out' argument
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(cloud_in);
    downsample.setLeafSize(0.2, 0.2 ,0.2);
    downsample.filter(*cloud_out);

  }

  void Homework3::normalsFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Compute normal vectors for the incoming point cloud
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    kd_tree_->setInputCloud(cloud_in);
    normal_estimator.setSearchMethod(kd_tree_);
    normal_estimator.setInputCloud(cloud_in);
    normal_estimator.setKSearch(50);
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
      if (angle_up > 30 && angle_down > 30){
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

  void Homework3::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
  {
    // TODO: Implement Euclidean clustering here, dumping the array of separated clouds into the 'cluster_clouds' argument
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(5000);
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

  void Homework3::mergeClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
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

  void Homework3::generateBoundingBoxes(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds)
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

}
