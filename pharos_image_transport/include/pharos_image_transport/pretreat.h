#ifndef PHAROS_IMAGE_PRETREAT_H
#define PHAROS_IMAGE_PRETREAT_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <pharos_image_transport/PharosImageTransportData.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <pharos_image_transport/pharosImageTransportConfig.h>

// image_transport
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace pharos_image_transport
{
class Pretreat
{
 public:
  //! Constructor.
  explicit Pretreat(ros::NodeHandle nh);

  void ImageCallback(const sensor_msgs::Image::ConstPtr &img);

  image_transport::Subscriber sub;

  image_transport::Publisher pub;

 private:
  //! Callback function for dynamic reconfigure server.
  void configCallback(pharos_image_transport::pharosImageTransportConfig &config, uint32_t level);

  //! ROS node handle.
  ros::NodeHandle nh_;

  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<pharos_image_transport::pharosImageTransportConfig> dr_srv_;

  //! The actual message.
  std::string camera_;
  std::string message_;

  bool enable_;
  int crop_width_;
  int crop_height_;
  double resize_gain_;

  bool center_offset_;
  int x_offset_;
  int y_offset_;

  bool rotate_;
  bool mono_;
};
}

#endif  // NODE_EXAMPLE_TALKER_H
