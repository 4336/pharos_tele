#ifndef PHAROS_CAM_INFO_PUB_H
#define PHAROS_CAM_INFO_PUB_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

// image_transport
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

namespace pharos_image_transport
{
class CamInfo
{
 public:
  //! Constructor.
  explicit CamInfo(ros::NodeHandle nh);

  std::string input_cam_;
  std::string input_info_;
  std::string output_cam_;
  std::string frame_id_;

  sensor_msgs::CameraInfo cam_info_;

  sensor_msgs::CameraInfo info_;
  sensor_msgs::CameraInfo info_2;

  float fx_, fy_, cx_, cy_, Tx_, Ty_;
  float roi_x_, roi_y_, roi_x_offset_, roi_y_offset_;

 private:
  void ImageCallback(const sensor_msgs::Image::ConstPtr &img);
  void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info);

  image_transport::Subscriber sub;
  image_transport::Publisher pub;
  image_transport::Publisher pub2;
  ros::Subscriber sub_info;
  ros::Publisher pub_info;
  ros::Publisher pub_info2;

  //! Callback function for dynamic reconfigure server.
  // void configCallback(pharos_image_transport::pharosImageTransportConfig &config, uint32_t level);

  //! ROS node handle.
  ros::NodeHandle nh_;

  //! Dynamic reconfigure server.
  // dynamic_reconfigure::Server<pharos_image_transport::pharosImageTransportConfig> dr_srv_;

  //! The actual message.
};
}

#endif  // NODE_EXAMPLE_TALKER_H
