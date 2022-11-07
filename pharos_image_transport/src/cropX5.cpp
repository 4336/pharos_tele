#include <pharos_image_transport/cropX5.h>

namespace pharos_image_transport
{
CropX5::CropX5(ros::NodeHandle nh) : nh_(nh)
{
  ros::NodeHandle pnh("~");
  pnh.param("cropX5_num", cropX5_num_, cropX5_num_);
  pnh.param("cropX5_cnt", cropX5_cnt_, cropX5_cnt_);
  pnh.param("topic_name", topic_name_, topic_name_);
  pnh.param("cropX5_width", cropX5_width_, cropX5_width_);
  pnh.param("cropX5_height", cropX5_height_, cropX5_height_);


  image_transport::ImageTransport it(nh);
  //Pubscribe
  pub = it.advertise(topic_name_, 1);
  // Subscribe
  sub = it.subscribe("/front_cam/image_tele", 1, &CropX5::ImageCallback, this);

  // Create timer.
  // timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &CropX5::Callback, this);
}

void CropX5::ImageCallback(const sensor_msgs::Image::ConstPtr &img){
  cv_bridge::CvImagePtr cv_ptr;

  // ROS_INFO("Image(%d, %d)", img->width, img->height);

  try{
    if(img->encoding == "mono8")cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    else cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }catch( cv_bridge::Exception &e){
  ROS_ERROR("Error to convert:");
    return;
  }

  int cropX5_width = img->width/cropX5_num_;
  int cropX5_height = img->height;

  cv::Mat cropX5_img;
  cv::Rect bounds(0,0, img->width,img->height);
  cv::Rect r(cropX5_width*cropX5_cnt_,0, cropX5_width,cropX5_height); // partly outside
  cropX5_img = cv_ptr->image( r & bounds );

  sensor_msgs::ImagePtr imgPtr;
  imgPtr = cv_bridge::CvImage(std_msgs::Header(), img->encoding, cropX5_img).toImageMsg();
  if(enable_) pub.publish(imgPtr);

  // cv::imshow("image_show", resize_im);
  // cv::destroyWindow("gray_Show");
  cv::waitKey(1);
}
}
