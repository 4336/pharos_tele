#include <pharos_image_transport/cam_info_pub.h>

namespace pharos_image_transport
{
CamInfo::CamInfo(ros::NodeHandle nh) : nh_(nh)
{
  ros::NodeHandle pnh("~");
  // pnh.param("view", view_, view_);

  pnh.param<std::string>("input_camera", input_cam_, "/input_camera");
  pnh.param<std::string>("input_info", input_info_, "/input_info");

  pnh.param<std::string>("output_camera", output_cam_, "/output_camera");

  pnh.param<float>("roi_x", roi_x_, 0);
  pnh.param<float>("roi_y", roi_y_, 0);
  pnh.param<float>("roi_x_offset_", roi_x_offset_, 0);
  pnh.param<float>("roi_y_offset_", roi_y_offset_, 0);

  pnh.param<float>("fx", fx_, 1);
  pnh.param<float>("fy", fy_, 1);
  pnh.param<float>("cx", cx_, 1);
  pnh.param<float>("cy", cy_, 1);
  pnh.param<float>("Tx", Tx_, 0);
  pnh.param<float>("Ty", Ty_, 0);

  // cam_info_.distortion_model = "plumb_bob";

  // pnh.param<int>("roi/x_offset", cam_info_.roi.x_offset);
  // pnh.param<int>("roi/y_offset", cam_info_.roi.y_offset);
  // pnh.param<int>("roi/height", cam_info_.roi.height);
  // pnh.param<int>("roi/width", cam_info_.roi.width);
  // pnh.param<bool>("roi/do_rectify", cam_info_.roi.do_rectify);

  std::vector<double> V;
  pnh.getParam("distortion_coefficients/data", V);
  for(int i=0; i<V.size(); i++) cam_info_.D.push_back(V[i]);

  pnh.getParam("camera_matrix/data", V);
  for(int i=0; i<V.size(); i++) cam_info_.K[i] = V[i];

  pnh.getParam("rectification_matrix/data", V);
  for(int i=0; i<V.size(); i++) cam_info_.R[i] = V[i];

  pnh.getParam("projection_matrix/data", V);
  for(int i=0; i<V.size(); i++) cam_info_.P[i] = V[i];


  //Pubscribe
  image_transport::ImageTransport it(nh);
  image_transport::TransportHints Hints("raw");
  pub = it.advertise(output_cam_+"/image_raw", 1);
  pub2 = it.advertise(output_cam_+"2/image_raw", 1);
  pub_info = nh.advertise<sensor_msgs::CameraInfo>(output_cam_+"/camera_info", 1);
  pub_info2 = nh.advertise<sensor_msgs::CameraInfo>(output_cam_+"2/camera_info", 1);
  // Subscribe
  sub = it.subscribe(input_cam_, 1, &CamInfo::ImageCallback, this, Hints);
  sub_info = nh.subscribe(input_info_, 1, &CamInfo::InfoCallback, this);
}

void CamInfo::ImageCallback(const sensor_msgs::Image::ConstPtr &img){

  // cam_info_.width = img->width;
  // cam_info_.height = img->height;
  // cam_info_.header = img->header;
  // pub_info.publish(cam_info_);

  pub.publish(*img);
  pub_info.publish(info_);
  
  static float delay = 0;
  static float delay_ = -1;
  nh_.getParam("/station/delay", delay);
  if(delay!=delay_){
    ROS_WARN("Delay Changed to %f", delay);
    delay_ = delay;
  }
  if(delay == 0){
    sensor_msgs::Image img2 = *img;
    img2.header.frame_id += "2";
    // img2.header.stamp = ros::Time::now();
    pub2.publish(img2);
  }

  pub_info2.publish(info_2);
}

void CamInfo::InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info){
  info_ = *info;
  info_2 = *info;
  sensor_msgs::RegionOfInterest roi;
  info_.roi = roi;
  info_.roi.x_offset = 0;
  info_.roi.y_offset = 90;
  info_.roi.width = 768;
  info_.roi.height = 108;
  info_.P[0] *= 1;//fx
  info_.P[5] *= 1;//fy

  info_.P[2] *= 1;//cx
  info_.P[6] *= 1;//cy

  info_.P[3] += 0;//Tx
  info_.P[7] += 0;//Ty


  info_2.roi = roi;

  info_2.roi.width = roi_x_;
  info_2.roi.height = roi_y_;
  info_2.roi.x_offset = roi_x_offset_;
  info_2.roi.y_offset = roi_y_offset_;

  info_2.P[0] *= fx_;//fx
  info_2.P[5] *= fy_;//fy

  info_2.P[2] *= cx_;//cx
  info_2.P[6] *= cy_;//cy

  info_2.P[3] += Tx_;//Tx
  info_2.P[7] += Ty_;//Ty
  info_2.header.frame_id += "2";
  std::cout<<"camera_info init\n";
  sub_info.shutdown();
}
}
