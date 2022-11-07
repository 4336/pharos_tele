#include <pharos_image_transport/pretreat.h>

namespace pharos_image_transport
{
Pretreat::Pretreat(ros::NodeHandle nh) : nh_(nh), camera_("front_cam"),
 message_("image_raw"), crop_width_(1920), crop_height_(1080), resize_gain_(1),
  enable_(true), mono_(true)
{
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<pharos_image_transport::pharosImageTransportConfig>::CallbackType cb;
  cb = boost::bind(&Pretreat::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  double rate = 1.0;

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("enable", enable_, enable_);
  pnh.param("camera", camera_, camera_);
  pnh.param("message", message_, message_);
  pnh.param("rate", rate, rate);
  pnh.param("crop_width", crop_width_, crop_width_);
  pnh.param("crop_height", crop_height_, crop_height_);
  pnh.param("resize_gain", resize_gain_, resize_gain_);
  pnh.param("x_offset", x_offset_, x_offset_);
  pnh.param("y_offset", y_offset_, y_offset_);
  pnh.param("center_offset", center_offset_, center_offset_);
  pnh.param("rotate", rotate_, rotate_);
  pnh.param("mono", mono_, mono_);


  image_transport::ImageTransport it(nh);
  //Pubscribe
  pub = it.advertise(camera_+"/image_tele", 1);
  // Subscribe
  sub = it.subscribe(camera_+message_, 1, &Pretreat::ImageCallback, this);

  // Create timer.
  // timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &Pretreat::Callback, this);
}

void Pretreat::configCallback(pharos_image_transport::pharosImageTransportConfig &config, uint32_t level __attribute__((unused)))
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  enable_ = config.enable;
  message_ = config.message;
  crop_width_ = config.crop_width;
  crop_height_ = config.crop_height;
  resize_gain_ = config.resize_gain;
  x_offset_ = config.x_offset;
  y_offset_ = config.y_offset;
  center_offset_ = config.center_offset;
  mono_ = config.mono;
  rotate_ = config.rotate;
}

void Pretreat::ImageCallback(const sensor_msgs::Image::ConstPtr &img){
  int sum=0;
  for(int i=0; i<10; i++){
    sum+=img->data[i];
  }
  if(sum==0) return;
  // std::cout<<size(img->data)<<"\n";
  //cv_bridge
  cv_bridge::CvImagePtr cv_ptr;

  // ROS_INFO("Image(%d, %d)", img->width, img->height);

  try{
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }catch( cv_bridge::Exception &e){
  ROS_ERROR("Error to convert:");
    return;
  }

  // Grayscale
  cv::Mat gray_im;
  if(mono_) cv::cvtColor( cv_ptr->image, gray_im, CV_RGB2GRAY );
  else gray_im = cv_ptr->image;

  //Rotate 180
  cv::Mat rotate_im;
  if(rotate_){
    cv::rotate(gray_im, rotate_im, 1);
  }else rotate_im = gray_im; // cropped to fit image

  //ROI
  cv::Mat roi_im;
  cv::Rect bounds(0,0,rotate_im.cols,rotate_im.rows);
  if(center_offset_){
    cv::Rect r1((rotate_im.cols-crop_width_)/2,(rotate_im.rows-crop_height_)/2, crop_width_,crop_height_); // partly outside
    roi_im = rotate_im( r1 & bounds );
  }else{
    if( x_offset_ > rotate_im.cols-crop_width_ ) x_offset_ = rotate_im.cols-crop_width_;
    if( y_offset_ > rotate_im.rows-crop_height_ ) y_offset_ = rotate_im.rows-crop_height_;
    cv::Rect r2(x_offset_,rotate_im.rows-crop_height_-y_offset_, crop_width_,crop_height_); // partly outside
    roi_im = rotate_im( r2 & bounds );
  }

  //resize
  cv::Mat resize_im = roi_im;
  int w = crop_width_*resize_gain_;
  int h = crop_height_*resize_gain_;
  cv::resize( roi_im, resize_im, cv::Size( w, h ), CV_INTER_NN );

  sensor_msgs::ImagePtr imgPtr;
  //Publish
  if(mono_) imgPtr = cv_bridge::CvImage(std_msgs::Header(), "mono8", resize_im).toImageMsg();
  else imgPtr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resize_im).toImageMsg();
  if(enable_) pub.publish(imgPtr); 

  // cv::imshow("image_show", resize_im);
  // // cv::destroyWindow("gray_Show");
  // cv::waitKey(1);
}
}
