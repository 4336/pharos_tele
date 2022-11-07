#ifndef PHAROS_STAMPED_VIEW_H
#define PHAROS_STAMPED_VIEW_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

// image_transport
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace pharos_image_transport
{
class StampedView
{
 public:
  //! Constructor.
  explicit StampedView(ros::NodeHandle nh, std::string topic);

  cv_bridge::CvImagePtr cv_ptr;

  bool init_;
  std::string topic_;
  cv::Mat mat_img;

 private:
  void ImageCallback(const sensor_msgs::Image::ConstPtr &img);

  image_transport::Subscriber sub;

  //! Callback function for dynamic reconfigure server.
  // void configCallback(pharos_image_transport::pharosImageTransportConfig &config, uint32_t level);

  //! ROS node handle.
  ros::NodeHandle nh_;

  //! Dynamic reconfigure server.
  // dynamic_reconfigure::Server<pharos_image_transport::pharosImageTransportConfig> dr_srv_;

  //! The actual message.
  bool enable_;
  int crop_num_;
  int crop_cnt_;
  int _width_;
  int crop_height_;
};
}

#endif  // NODE_EXAMPLE_TALKER_H
