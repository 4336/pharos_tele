#ifndef PHAROS_IMAGE_CROPX5_H
#define PHAROS_IMAGE_CROPX5_H

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
class CropX5
{
 public:
  //! Constructor.
  explicit CropX5(ros::NodeHandle nh);

  void ImageCallback(const sensor_msgs::Image::ConstPtr &img);

  image_transport::Subscriber sub;

  image_transport::Publisher pub0;
  image_transport::Publisher pub1;
  image_transport::Publisher pub2;
  image_transport::Publisher pub3;
  image_transport::Publisher pub4;

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
  int cropX5_num_;
  int cropX5_cnt_;
  int cropX5_width_;
  int cropX5_height_;
};
}

#endif  // NODE_EXAMPLE_TALKER_H
