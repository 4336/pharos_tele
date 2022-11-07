#ifndef PHAROS_IMAGE_CROP_H
#define PHAROS_IMAGE_CROP_H

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
class Crop
{
 public:
  //! Constructor.
  explicit Crop(ros::NodeHandle nh);

  void ImageCallback(const sensor_msgs::Image::ConstPtr &img);

  image_transport::Subscriber sub;

  image_transport::Publisher pub;

 private:
  //! Callback function for dynamic reconfigure server.
  // void configCallback(pharos_image_transport::pharosImageTransportConfig &config, uint32_t level);

  //! ROS node handle.
  ros::NodeHandle nh_;

  //! Dynamic reconfigure server.
  // dynamic_reconfigure::Server<pharos_image_transport::pharosImageTransportConfig> dr_srv_;

  //! The actual message.
  std::string topic_name_;

  bool enable_;
  int crop_num_;
  int crop_cnt_;
  int crop_width_;
  int crop_height_;
};
}

#endif  // NODE_EXAMPLE_TALKER_H
