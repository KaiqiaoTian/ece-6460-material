#include "EkfExample.hpp"

using namespace Eigen;

namespace ece6460_ekf_example {

EkfExample::EkfExample(ros::NodeHandle n, ros::NodeHandle pn)
{
  // Set reference coordinates
  double ref_utm_x = 0;
  double ref_utm_y = 0;
  double ref_lat = 0;
  double ref_lon = 0;

  bool found_lat = pn.getParam("ref_lat", ref_lat);
  bool found_lon = pn.getParam("ref_lon", ref_lon);
  if (!found_lat || !found_lon) {
    ROS_ERROR("Latitude or longitude reference not set! Bad things will happen");
  } else {
    std::string utm_zone;
    gps_common::LLtoUTM(ref_lat, ref_lon, ref_utm_y, ref_utm_x, utm_zone);
  }
  ref_utm_vect_ = tf::Vector3(ref_utm_x, ref_utm_y, 0);

  // Base sampling time of the filter
  double sample_time = 0.01;

  // Subscribe to input data, advertise path, and set up main filter timer
  sub_fix_ = n.subscribe("fix", 1, &EkfExample::recvFix, this);
  sub_twist_ = n.subscribe("twist", 1, &EkfExample::recvTwist, this);
  pub_odom_ = n.advertise<nav_msgs::Odometry>("filter_odom", 1);
  timer_ = n.createTimer(ros::Duration(sample_time), &EkfExample::timerCallback, this);

  // Set up dynamic reconfigure server
  srv_.setCallback(boost::bind(&EkfExample::reconfig, this, _1, _2));

  X_.setZero();
  P_.setIdentity();
}

void EkfExample::recvTwist(const geometry_msgs::TwistStampedConstPtr& msg)
{
  updateFilterTwist(msg->header.stamp, msg->twist);
}

void EkfExample::recvFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
  double current_utm_x;
  double current_utm_y;
  std::string utm_zone;
  gps_common::LLtoUTM(msg->latitude, msg->longitude, current_utm_y, current_utm_x, utm_zone);

  tf::Vector3 current_position = tf::Vector3(current_utm_x, current_utm_y, 0) - ref_utm_vect_;
  updateFilterGPS(msg->header.stamp, current_position);

  // Update TF transform with raw gps output
  geometry_msgs::TransformStamped raw_gps_transform;
  raw_gps_transform.header.stamp = msg->header.stamp;
  raw_gps_transform.header.frame_id = "map";
  raw_gps_transform.child_frame_id = "raw";
  raw_gps_transform.transform.rotation.w = 1.0;
  raw_gps_transform.transform.translation.x = current_position.x();
  raw_gps_transform.transform.translation.y = current_position.y();
  raw_gps_transform.transform.translation.z = 0;
  broadcaster_.sendTransform(raw_gps_transform);
}

StateVector EkfExample::statePrediction(double dt, const StateVector& old_state)
{
  double old_heading = old_state(2);
  double old_speed = old_state(3);
  double old_yaw_rate = old_state(4);

  StateVector new_state;
  new_state(0) = old_state(0) + dt * old_speed * cos(old_heading);
  new_state(1) = old_state(1) + dt * old_speed * sin(old_heading);
  new_state(2) = old_state(2) + dt * old_yaw_rate;
  new_state(3) = old_state(3);
  new_state(4) = old_state(4);
  return new_state;
}

StateMatrix EkfExample::stateJacobian(double dt, const StateVector& state)
{
  double heading_est = state(2);
  double speed_est = state(3);

  // Populate state Jacobian matrix with current state values
  StateMatrix A;
  A.row(0) << 1, 0, -dt * speed_est * sin(heading_est), dt * cos(heading_est), 0;
  A.row(1) << 0, 1,  dt * speed_est * cos(heading_est), dt * sin(heading_est), 0;
  A.row(2) << 0, 0, 1, 0, dt;
  A.row(3) << 0, 0, 0, 1, 0;
  A.row(4) << 0, 0, 0, 0, 1;
  return A;
}

StateMatrix EkfExample::covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov)
{
  StateMatrix new_cov = A * old_cov * A.transpose() + Q;
  return new_cov;
}

void EkfExample::updateFilterNone(const ros::Time& current_time)
{
  if (estimate_stamp_ == ros::Time(0)) {
    ROS_WARN_THROTTLE(1.0, "Waiting for first GPS fix, ignoring this update");
    return;
  }

  // Compute amount of time to advance the state prediction
  double dt = (current_time - estimate_stamp_).toSec();

  // Propagate estimate prediction and update estimate with result
  StateMatrix A = stateJacobian(dt, X_);
  X_ = statePrediction(dt, X_);
  P_ = covPrediction(A, Q_, P_);
  estimate_stamp_ = current_time;

  // Wrap heading estimate into the range -pi to pi
  if (X_(2) > M_PI) {
    X_(2) -= 2 * M_PI;
  } else if (X_(2) < -M_PI) {
    X_(2) += 2 * M_PI;
  }
}

void EkfExample::updateFilterGPS(const ros::Time& current_time, const tf::Vector3& position)
{
  // If uninitialied, set filter state to this GPS position measurement
  if (estimate_stamp_ == ros::Time(0)) {
    X_ << position.x(), position.y(), 0.0, 0.0, 0.0;
    P_.setIdentity();
    estimate_stamp_ = current_time;
    return;
  }

  // Compute amount of time to advance the state prediction
  double dt = (current_time - estimate_stamp_).toSec();

  // Propagate estimate prediction and store in predicted variables
  StateMatrix A = stateJacobian(dt, X_);
  StateVector predicted_state = statePrediction(dt, X_);
  StateMatrix predicted_cov = covPrediction(A, Q_, P_);

  // Construct C matrix for a GPS update (X and Y position measurements)
  Eigen::Matrix<double, 2, 5> C;
  C << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;
  
  // Use C and predicted state to compute expected measurement
  Eigen::Matrix<double, 2, 1> expected_meas = C * predicted_state;

  // Put GPS measurements in an Eigen object
  Eigen::Matrix<double, 2, 1> real_meas;
  real_meas << position.x(), position.y();

  // Define R matrix for the GPS measurements
  Eigen::Matrix<double, 2, 2> R;
  R << cfg_.r_gps * cfg_.r_gps, 0,
       0, cfg_.r_gps * cfg_.r_gps;

  // Compute Kalman gain
  Eigen::Matrix<double, 5, 2> K;
  K = predicted_cov * C.transpose() * (C * predicted_cov * C.transpose() + R).inverse();

  // Update filter estimate based on difference between actual and expected measurements
  X_ = predicted_state + K * (real_meas - expected_meas);
  P_ = (StateMatrix::Identity() - K * C) * predicted_cov;

  // If using a measurement from the past (dt < 0), re-propagate filter to the current estimate stamp
  if (dt < 0) {
    A = stateJacobian(-dt, X_);
    X_ = statePrediction(-dt, X_);
    P_ = covPrediction(A, Q_, P_);
  }

  // Wrap heading estimate into the range -pi to pi
  if (X_(2) > M_PI) {
    X_(2) -= 2 * M_PI;
  } else if (X_(2) < -M_PI) {
    X_(2) += 2 * M_PI;
  }

  // Set estimate time stamp to the measurement's time, unless it was from the past
  if (dt >= 0) {
    estimate_stamp_ = current_time;
  }
}

void EkfExample::updateFilterTwist(const ros::Time& current_time, const geometry_msgs::Twist& twist)
{
  if (estimate_stamp_ == ros::Time(0)) {
    ROS_WARN_THROTTLE(1.0, "Waiting for first GPS fix, ignoring this update");
    return;
  }

  // Compute amount of time to advance the state prediction
  double dt = (current_time - estimate_stamp_).toSec();

  // Propagate estimate prediction and store in predicted variables
  StateMatrix A = stateJacobian(dt, X_);
  StateVector predicted_state = statePrediction(dt, X_);
  StateMatrix predicted_cov = covPrediction(A, Q_, P_);

  // Construct C matrix for a GPS update (speed and yaw rate measurements)
  Eigen::Matrix<double, 2, 5> C;
  C << 0, 0, 0, 1, 0,
       0, 0, 0, 0, 1;
  
  // Use C and predicted state to compute expected measurement
  Eigen::Matrix<double, 2, 1> expected_meas = C * predicted_state;

  // Put GPS measurements in an Eigen object
  Eigen::Matrix<double, 2, 1> real_meas;
  real_meas << twist.linear.x, twist.angular.z;

  // Define R matrix for the GPS measurements
  Eigen::Matrix<double, 2, 2> R;
  R << cfg_.r_speed * cfg_.r_speed, 0,
       0, cfg_.r_yaw_rate * cfg_.r_yaw_rate;

  // Compute Kalman gain
  Eigen::Matrix<double, 5, 2> K;
  K = predicted_cov * C.transpose() * (C * predicted_cov * C.transpose() + R).inverse();

  // Update filter estimate based on difference between actual and expected measurements
  X_ = predicted_state + K * (real_meas - expected_meas);
  P_ = (StateMatrix::Identity() - K * C) * predicted_cov;
  
  // Wrap heading estimate into the range -pi to pi
  if (X_(2) > M_PI) {
    X_(2) -= 2 * M_PI;
  } else if (X_(2) < -M_PI) {
    X_(2) += 2 * M_PI;
  }

  // Set estimate time stamp to the measurement's time
  estimate_stamp_ = current_time;
}

void EkfExample::timerCallback(const ros::TimerEvent& event)
{
  updateFilterNone(event.current_real);

  // Update TF transform with current estimate
  geometry_msgs::TransformStamped ekf_transform;
  ekf_transform.header.stamp = event.current_real;
  ekf_transform.header.frame_id = "map";
  ekf_transform.child_frame_id = "filter";
  ekf_transform.transform.rotation = tf::createQuaternionMsgFromYaw(X_(2));
  ekf_transform.transform.translation.x = X_(0);
  ekf_transform.transform.translation.y = X_(1);
  ekf_transform.transform.translation.z = 0;
  broadcaster_.sendTransform(ekf_transform);

  // Publish odometry to visualize past history
  nav_msgs::Odometry odom_msg;
  odom_msg.header = ekf_transform.header;
  odom_msg.child_frame_id = "filter";
  odom_msg.pose.pose.position.x = X_(0);
  odom_msg.pose.pose.position.y = X_(1);
  odom_msg.pose.pose.orientation = ekf_transform.transform.rotation;
  odom_msg.twist.twist.linear.x = X_(3);
  odom_msg.twist.twist.angular.z = X_(4);
  pub_odom_.publish(odom_msg);
}

void EkfExample::reconfig(EkfExampleConfig& config, uint32_t level)
{
  Q_.setZero();
  Q_(0, 0) = config.q_pos * config.q_pos;
  Q_(1, 1) = config.q_pos * config.q_pos;
  Q_(2, 2) = config.q_heading * config.q_heading;
  Q_(3, 3) = config.q_speed * config.q_speed;
  Q_(4, 4) = config.q_yaw_rate * config.q_yaw_rate;

  cfg_ = config;
}

}
