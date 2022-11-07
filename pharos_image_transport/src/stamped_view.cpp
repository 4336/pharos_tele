#include <pharos_image_transport/stamped_view.h>

namespace pharos_image_transport
{
StampedView::StampedView(ros::NodeHandle nh, std::string topic)
{
  init_ = false;
  topic_ = topic;

  ros::NodeHandle pnh("~");
  // pnh.param("view", view_, view_);

  image_transport::ImageTransport it(nh);
  image_transport::TransportHints Hints("compressed", ros::TransportHints().tcpNoDelay());
  //Pubscribe
  // pub = it.advertise(topic_name_, 1);
  // Subscribe
  sub = it.subscribe(topic, 1, &StampedView::ImageCallback, this, Hints);
}

void StampedView::ImageCallback(const sensor_msgs::Image::ConstPtr &img){

  try{
    if(img->encoding == "mono8")cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    else cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    mat_img = cv_ptr->image;
    init_ = true;
    std::cout << ros::Time::now() - img->header.stamp << std::endl;
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
