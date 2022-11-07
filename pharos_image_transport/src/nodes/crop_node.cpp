#include <pharos_image_transport/crop.h>

// image_transport
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "crop");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  pharos_image_transport::Crop node(nh);

  // Let ROS handle all callbacks.
  ros::spin();

  return 0;
}  // end main()
