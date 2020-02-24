#include "HsvExample.hpp"

namespace ece6460_opencv_example {

HsvExample::HsvExample(ros::NodeHandle n, ros::NodeHandle pn)
{
  std::string filename = ros::package::getPath("ece6460_opencv_example") + "/hsv.png";
  raw_img_ = cv::imread(filename.c_str(), CV_LOAD_IMAGE_COLOR);
  cv::cvtColor(raw_img_, raw_hsv_img_, CV_BGR2HSV);

  refresh_timer_ = n.createTimer(ros::Duration(0.05), &HsvExample::timerCallback, this);

  srv_.setCallback(boost::bind(&HsvExample::reconfig, this, _1, _2));

  cv::namedWindow("H", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("S", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("V", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("H_thres", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("S_thres", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("V_thres", CV_WINDOW_AUTOSIZE);
}

void HsvExample::reconfig(HsvExampleConfig& config, uint32_t level)
{
  cfg_ = config;

  // Split HSV image into separate single-channel images for H, S, and V
  // and store each in dedicated variables
  cv::split(raw_hsv_img_, split_img);
  cv::Mat hue_img = split_img[0];
  cv::Mat sat_img = split_img[1];
  cv::Mat val_img = split_img[2];

  // Threshold saturation and put output binary image in sat_thres
  cv::threshold(sat_img, sat_thres, cfg_.s_thres, 255, CV_THRESH_BINARY);

  // Threshold value and put output binary image in val_thres
  cv::threshold(val_img, val_thres, cfg_.v_thres, 255, CV_THRESH_BINARY);

  // Threshold hue and put output binary image in hue_thres
  cv::Mat t1;
  cv::Mat t2;
  int h_pos_edge = cfg_.h_center + cfg_.h_width;
  int h_neg_edge = cfg_.h_center - cfg_.h_width;
  if (h_pos_edge > 180) {
    cv::threshold(hue_img, t1, h_pos_edge - 180, 255, CV_THRESH_BINARY_INV);  
    cv::threshold(hue_img, t2, cfg_.h_center - cfg_.h_width, 255, CV_THRESH_BINARY);  
    cv::bitwise_or(t1, t2, hue_thres);
  } else if (h_neg_edge < 0) {
    cv::threshold(hue_img, t1, h_neg_edge + 180, 255, CV_THRESH_BINARY);  
    cv::threshold(hue_img, t2, cfg_.h_center + cfg_.h_width, 255, CV_THRESH_BINARY_INV);  
    cv::bitwise_or(t1, t2, hue_thres);
  } else {
    cv::threshold(hue_img, t1, cfg_.h_center - cfg_.h_width, 255, CV_THRESH_BINARY);
    cv::threshold(hue_img, t2, cfg_.h_center + cfg_.h_width, 255, CV_THRESH_BINARY_INV);
    cv::bitwise_and(t1, t2, hue_thres);
  }
}

void HsvExample::timerCallback(const ros::TimerEvent& event)
{
  cv::imshow("H", split_img[0]);
  cv::waitKey(1);
  cv::imshow("S", split_img[1]);
  cv::waitKey(1);
  cv::imshow("V", split_img[2]);
  cv::waitKey(1);

  cv::imshow("H_thres", hue_thres);
  cv::waitKey(1);
  cv::imshow("S_thres", sat_thres);
  cv::waitKey(1);
  cv::imshow("V_thres", val_thres);
  cv::waitKey(1);
}

}
