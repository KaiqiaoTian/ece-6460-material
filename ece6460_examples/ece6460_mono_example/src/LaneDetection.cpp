#include "LaneDetection.hpp"

#define DEBUG 1

using namespace cv;

namespace ece6460_mono_example
{

LaneDetection::LaneDetection(ros::NodeHandle n, ros::NodeHandle pn) :
  listener_(buffer_),
  kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
{
  sub_cam_info_ = n.subscribe("camera_info", 1, &LaneDetection::recvCameraInfo, this);
  sub_image_ = n.subscribe("image_rect_color", 1, &LaneDetection::recvImage, this);
  pub_markers_ = n.advertise<visualization_msgs::MarkerArray>("projected_lines", 1);
  pub_cloud_ = n.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);

  srv_.setCallback(boost::bind(&LaneDetection::reconfig, this, _1, _2));
  looked_up_camera_transform_ = false;

  pn.param("camera_name", camera_name_, std::string("front_camera"));

#if DEBUG
  namedWindow("Binary", CV_WINDOW_NORMAL);
#endif
}

// This function is called whenever a new image is received from either
// the live running camera driver, a bag file recording, or the simulated
// image coming from Gazebo
void LaneDetection::recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  // Do nothing until the coordinate transform from footprint to camera is valid,
  // because otherwise there is no point in detecting a lane!
  if (!looked_up_camera_transform_) {
    try {
      geometry_msgs::TransformStamped transform = buffer_.lookupTransform("base_footprint", camera_name_ + "_optical", msg->header.stamp);
      tf2::convert(transform.transform, camera_transform_);
      looked_up_camera_transform_ = true; // Once the lookup is successful, there is no need to keep doing the lookup
                                          // because the transform is constant
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    }
    return;
  }

  // Convert ROS image message into an OpenCV Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  Mat raw_img = cv_ptr->image;
  Mat bin_img;
  segmentImage(raw_img, bin_img);
#if DEBUG
  imshow("Binary", bin_img);
  waitKey(1);
#endif

  // Create pinhole camera model instance and load
  // its parameters from the camera info
  // generated using the checkerboard calibration program
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);

  // Project points from 2D pixel coordinates into 3D where it intersects
  // the ground plane, and put them in a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Project every fourth row of the image to save some computational resources
  for (int i = 0; i < bin_img.rows; i += 4) {
    for (int j = 0; j < bin_img.cols; j++) {
      if (bin_img.at<uint8_t>(i, j) == 255) {
        // We found a white pixel corresponding to a lane marking. Project to ground
        // and add to point cloud
        geometry_msgs::Point proj_p = projectPoint(model, cv::Point(j, i));
        pcl::PointXYZ p;
        p.x = proj_p.x;
        p.y = proj_p.y;
        p.z = proj_p.z;
        bin_cloud->points.push_back(p);
      }
    }
  }
  
  // Publish point cloud to visualize in Rviz
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*bin_cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_footprint";
  cloud_msg.header.stamp = msg->header.stamp;
  pub_cloud_.publish(cloud_msg);

  // Use Euclidean clustering to group dashed lines together
  // and separate each distinct line into separate point clouds
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cfg_.cluster_tol);
  ec.setMinClusterSize(cfg_.min_cluster_size);
  ec.setMaxClusterSize(cfg_.max_cluster_size);
  kd_tree_->setInputCloud(bin_cloud);
  ec.setSearchMethod(kd_tree_);
  ec.setInputCloud(bin_cloud);
  ec.extract(cluster_indices);

  // Split clusters into separate point clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
  for (auto indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*bin_cloud, indices, *cluster);
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    cluster_clouds.push_back(cluster);
  }

  // Construct polynomial curve fits to each cluster cloud
  std::vector<CurveFit> curves;
  for (size_t i = 0; i < cluster_clouds.size(); i++) {
    CurveFit new_curve;
    // TODO: Quantify quality of the curve fit
    if (!fitPoints(cluster_clouds[i], cfg_.fit_order, new_curve)) {
      ROS_WARN("Failed curve fit");
      continue;
    }

    if (checkCurve(cluster_clouds[i], new_curve)) {
      curves.push_back(new_curve);
    }
  }

  // Construct Rviz marker output to visualize curve fit
  visualization_msgs::MarkerArray marker_msg;
  visualization_msgs::Marker viz_marker;
  viz_marker.header.frame_id = "base_footprint";
  viz_marker.header.stamp = msg->header.stamp;
  viz_marker.action = visualization_msgs::Marker::ADD;
  viz_marker.pose.orientation.w = 1;
  viz_marker.id = 0;

  // Use the LINE_STRIP type to display a line connecting each point
  viz_marker.type = visualization_msgs::Marker::LINE_STRIP;

  // Red
  viz_marker.color.a = 1.0;
  viz_marker.color.r = 1.0;
  viz_marker.color.g = 0.0;
  viz_marker.color.b = 0.0;

  // 0.1 meters thick line
  viz_marker.scale.x = 0.1;

  // Sample polynomial and add line strip marker to array
  // for each separate cluster
  for (auto& curve : curves) {
    visualizePoints(curve, viz_marker.points);
    marker_msg.markers.push_back(viz_marker);
    viz_marker.id++;
  }

  // Delete markers to avoid ghost markers from lingering if
  // the number of markers being published changes
  visualization_msgs::MarkerArray delete_markers;
  delete_markers.markers.resize(1);
  delete_markers.markers[0].action = visualization_msgs::Marker::DELETEALL;
  pub_markers_.publish(delete_markers);

  // Publish for visualization
  pub_markers_.publish(marker_msg);
}

void LaneDetection::segmentImage(const Mat& raw_img, Mat& bin_img)
{
  // Convert to HSV colorspace
  Mat raw_hsv;
  cvtColor(raw_img, raw_hsv, CV_BGR2HSV);

  // Split HSV image into separate single-channel images for H, S, and V
  // and store each in dedicated variables
  std::vector<Mat> split_img;
  split(raw_hsv, split_img);
  Mat hue_img = split_img[0];
  Mat sat_img = split_img[1];
  Mat val_img = split_img[2];

  // Detect white lane marking pixels in the image
  Mat white_bin_img;
  detectWhite(sat_img, val_img, white_bin_img);

  // Detect yellow lane marking pixels in the image
  Mat yellow_bin_img;
  detectYellow(hue_img, sat_img, yellow_bin_img);

  // Combine yellow and white detection with bitwise OR,
  // also applying a mask to ignore the hood of the car
  // that is in frame
  Mat mask = Mat::ones(white_bin_img.size(), CV_8U);
  mask(Rect(0, cfg_.mask_height, mask.cols, mask.rows - cfg_.mask_height - 1)) = 0;
  bitwise_or(white_bin_img, yellow_bin_img, bin_img, mask);

  // Apply Canny edge detection to greatly reduce the number
  // of detected pixels
  Canny(bin_img, bin_img, 2, 4);
}

void LaneDetection::detectWhite(const cv::Mat& sat_img, const cv::Mat& val_img, cv::Mat& white_bin_img)
{
  // Apply threshold to generate a binary value image. White lines have
  // higher value than the road
  Mat v_thres;
  threshold(val_img, v_thres, cfg_.val_thres, 255, CV_THRESH_BINARY);

  // Apply inverse threshold to generate a binary saturation image. We want
  // to throw out high saturation pixels because white has very low saturation
  Mat s_thres;
  threshold(sat_img, s_thres, cfg_.sat_thres, 255, CV_THRESH_BINARY_INV);

  // Bitwise AND to make sure only pixels that satisfy both value and saturation
  // thresholds make it out
  bitwise_and(v_thres, s_thres, white_bin_img);
}

void LaneDetection::detectYellow(const cv::Mat& hue_img, const cv::Mat& sat_img, cv::Mat& yellow_bin_img)
{
  // Threshold Hue to extract the yellow color
  Mat t1;
  Mat t2;
  Mat h_thres;
  int h_pos_edge = cfg_.h_center + cfg_.h_width;
  int h_neg_edge = cfg_.h_center - cfg_.h_width;
  if (h_pos_edge > 180) {
    threshold(hue_img, t1, h_pos_edge - 180, 255, CV_THRESH_BINARY_INV);  
    threshold(hue_img, t2, cfg_.h_center - cfg_.h_width, 255, CV_THRESH_BINARY);  
    bitwise_and(t1, t2, h_thres);
  } else if (h_neg_edge < 0) {
    threshold(hue_img, t1, h_neg_edge + 180, 255, CV_THRESH_BINARY);  
    threshold(hue_img, t2, cfg_.h_center + cfg_.h_width, 255, CV_THRESH_BINARY_INV);  
    bitwise_and(t1, t2, h_thres);
  } else {
    threshold(hue_img, t1, cfg_.h_center - cfg_.h_width, 255, CV_THRESH_BINARY);
    threshold(hue_img, t2, cfg_.h_center + cfg_.h_width, 255, CV_THRESH_BINARY_INV);
    bitwise_and(t1, t2, h_thres);
  }

  // Threshold saturation
  Mat s_thres;
  threshold(sat_img, s_thres, cfg_.sat_thres, 255, CV_THRESH_BINARY);

  // Bitwise AND to make sure only pixels that satisfy both hue and saturation
  // thresholds make it out
  bitwise_and(h_thres, s_thres, yellow_bin_img);
}

// Project 2D pixel point 'p' into vehicle's frame and return as 3D point
geometry_msgs::Point LaneDetection::projectPoint(const image_geometry::PinholeCameraModel& model, const Point2d& p)
{
  // Convert the input pixel coordinates into a 3d ray, where x and y are projected to the point where z is equal to 1.0
  cv::Point3d cam_frame_ray = model.projectPixelTo3dRay(p);
  
  // Represent camera frame ray in footprint frame
  tf2::Vector3 footprint_frame_ray = camera_transform_.getBasis() * tf2::Vector3(cam_frame_ray.x, cam_frame_ray.y, cam_frame_ray.z);

  // Using the concept of similar triangles, scale the unit vector such that the end is on the ground plane.
  double s = -camera_transform_.getOrigin().z() / footprint_frame_ray.z();
  tf2::Vector3 ground_plane_ray = s * footprint_frame_ray;

  // Then add camera position offset to obtain the final coordinates in footprint frame
  tf2::Vector3 vehicle_frame_point = ground_plane_ray + camera_transform_.getOrigin();

  // Fill output point with the result of the projection
  geometry_msgs::Point point;
  point.x = vehicle_frame_point.x();
  point.y = vehicle_frame_point.y();
  point.z = vehicle_frame_point.z();
  return point;
}

// This function inputs a PCL point cloud and applies a least squares curve fit
// to the points which fits an optimal polynomial curve of the desired order
bool LaneDetection::fitPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int order, CurveFit& curve)
{
  // Check if it is mathematically possible to fit a curve to the data
  if (cloud->points.size() <= order) {
    return false;
  }

  Eigen::MatrixXd regression_matrix(cloud->points.size(), order + 1);
  Eigen::VectorXd y_samples(cloud->points.size());

  curve.min_x = INFINITY;
  curve.max_x = 0.0;
  for (int i = 0; i < cloud->points.size(); i++) {
    y_samples(i) = cloud->points[i].y;

    // Fill in row of regression matrix
    // [1, x, x^2, ..., x^N]
    double tx = 1.0;
    for (int j = 0; j <= order; j++) {
      regression_matrix(i, j) = tx;
      tx *= cloud->points[i].x;
    }

    // Compute the minimum value of x to constrain
    // the polynomial curve
    if (cloud->points[i].x < curve.min_x) {
      curve.min_x = cloud->points[i].x;
    }

    // Compute the maximum value of x to constrain
    // the polynomial curve
    if (cloud->points[i].x > curve.max_x) {
      curve.max_x = cloud->points[i].x;
    }
  }

  // Invert regression matrix with left pseudoinverse operation
  Eigen::MatrixXd inv_regression_matrix = (regression_matrix.transpose() * regression_matrix).inverse() * regression_matrix.transpose();

  // Perform least squares estimation and obtain polynomial coefficients
  Eigen::VectorXd poly_coeff_matrix = inv_regression_matrix * y_samples;

  // Populate 'poly_coeff' field of the 'curve' argument output
  curve.poly_coeff.resize(poly_coeff_matrix.rows());
  for (size_t i = 0; i < poly_coeff_matrix.rows(); i++) {
    curve.poly_coeff[i] = poly_coeff_matrix(i);
  }

  return true; // Successful curve fit!
}

bool LaneDetection::checkCurve(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const CurveFit& curve)
{

  double rms;
  std::vector<double> error_samples;
  for (size_t i = 0; i < cloud->points.size(); i++) {
    double new_error;
    double y_hat = 0.0;
    double t = 1.0;
    for (size_t j = 0; j < curve.poly_coeff.size(); j++) {
      y_hat += curve.poly_coeff[j] * t;
      t *= cloud->points[i].x;
    }
    new_error = cloud->points[i].y - y_hat;
    error_samples.push_back(new_error);
  }

  double mean_square = 0;
  for (size_t i = 0; i < cloud->points.size(); i++) {
    mean_square += error_samples[i] * error_samples[i];
  }
  mean_square /= (double)cloud->points.size();

  rms = sqrt(mean_square);

  return rms < cfg_.rms_tolerance;
}

// This method samples a curve fit between its minimum and maximum valid values
// and outputs an array of geometry_msgs::Point to put into a Rviz marker message
void LaneDetection::visualizePoints(const CurveFit& curve, std::vector<geometry_msgs::Point>& points)
{
  points.clear();

  // Sample points at 0.05 meter resolution
  for (double x = curve.min_x; x <= curve.max_x; x += 0.05) {
    geometry_msgs::Point p;
    // Copy x value
    p.x = x;

    // Compute y value based on the order of the curve
    // y = a0 + a1 * x + a2 * x^2 + ... + aM * x^M
    double t = 1.0;
    for (size_t i = 0; i < curve.poly_coeff.size(); i++) {
      p.y += curve.poly_coeff[i] * t;
      t *= p.x;
    }

    points.push_back(p);
  }
}

void LaneDetection::recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  camera_info_ = *msg;
}

void LaneDetection::reconfig(LaneDetectionConfig& config, uint32_t level)
{
  cfg_ = config;
}

}