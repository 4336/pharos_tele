#include <pharos_image_transport/cam_info_pub.h>

// image_transport
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv)
{
  sensor_msgs::Image msg;

  // Set up ROS.
  ros::init(argc, argv, "cam_info");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.

  // Crop Callback
  pharos_image_transport::CamInfo info(nh);

  ros::spin();

  return 0;
}  // end main()
