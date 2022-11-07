#include <pharos_image_transport/stamped_view.h>

// image_transport
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  sensor_msgs::Image msg;

  // Set up ROS.
  ros::init(argc, argv, "pharos_view");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string image_;
  pnh.param<std::string>("image", image_, "/front_cam/image_raw");

  // Create a new node_example::Talker object.
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(image_ + "/image_view", 1);

  // Crop Callback
  pharos_image_transport::StampedView view(nh, image_);

  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    if(view.init_){
      sensor_msgs::ImagePtr imgPtr;
      imgPtr = cv_bridge::CvImage(std_msgs::Header(), view.cv_ptr->encoding, view.mat_img).toImageMsg();

      pub.publish(imgPtr);
      // std::cout<<"pub\n";

      cv::Mat mergeImg;
      cv::resize( view.mat_img, mergeImg, cv::Size( 3840, 1080 ), CV_INTER_NN );
      cv::imshow("merge_image", mergeImg);
      cv::waitKey(1);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}  // end main()
