#include <pharos_image_transport/view.h>

namespace pharos_image_transport
{
View::View(ros::NodeHandle nh, std::string topic) : nh_(nh)
{
  init_ = false;
  topic_ = topic;

  ros::NodeHandle pnh("~");
  // pnh.param("view", view_, view_);

  image_transport::ImageTransport it(nh);
  image_transport::TransportHints Hints("compressed", ros::TransportHints().udp());
  //Pubscribe
  // pub = it.advertise(topic_name_, 1);
  // Subscribe
  sub = it.subscribe(topic, 1, &View::ImageCallback, this, Hints);
}

void View::ImageCallback(const sensor_msgs::Image::ConstPtr &img){

  try{
    if(img->encoding == "mono8")cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    else cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    mat_img = cv_ptr->image;
    init_ = true;
    // std::cout<<topic_<<"\n";
  }catch( cv_bridge::Exception &e){
  ROS_ERROR("Error to convert:");
    return;
  }

  // cv::imshow("image_show", resize_im);
  // cv::destroyWindow("gray_Show");
  // cv::waitKey(1);
}
}
