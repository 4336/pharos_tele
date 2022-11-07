#include <pharos_image_transport/crop.h>

namespace pharos_image_transport
{
Crop::Crop(ros::NodeHandle nh) : nh_(nh)
{
  ros::NodeHandle pnh("~");
  pnh.param("crop_num", crop_num_, crop_num_);
  pnh.param("crop_cnt", crop_cnt_, crop_cnt_);
  pnh.param("topic_name", topic_name_, topic_name_);
  pnh.param("crop_width", crop_width_, crop_width_);
  pnh.param("crop_height", crop_height_, crop_height_);


  image_transport::ImageTransport it(nh);
  //Pubscribe
  pub = it.advertise(topic_name_, 1);
  // Subscribe
  sub = it.subscribe("/front_cam/image_tele", 1, &Crop::ImageCallback, this);

  // Create timer.
  // timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &Crop::Callback, this);
}

void Crop::ImageCallback(const sensor_msgs::Image::ConstPtr &img){
  cv_bridge::CvImagePtr cv_ptr;

  // ROS_INFO("Image(%d, %d)", img->width, img->height);

  try{
    if(img->encoding == "mono8")cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    else cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }catch( cv_bridge::Exception &e){
  ROS_ERROR("Error to convert:");
    return;
  }

  int crop_width = img->width/crop_num_;
  int crop_height = img->height;

  cv::Mat crop_img;
  cv::Rect bounds(0,0, img->width,img->height);
  cv::Rect r(crop_width*crop_cnt_,0, crop_width,crop_height); // partly outside
  crop_img = cv_ptr->image( r & bounds );

  sensor_msgs::ImagePtr imgPtr;
  imgPtr = cv_bridge::CvImage(std_msgs::Header(), img->encoding, crop_img).toImageMsg();
  if(enable_) pub.publish(imgPtr);

  // cv::imshow("image_show", resize_im);
  // cv::destroyWindow("gray_Show");
  cv::waitKey(1);
}
}
