#include <pharos_image_transport/view.h>

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
  ros::init(argc, argv, "view");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/front_cam/image_merge", 1);

  int width;
  nh.getParam("/front_cam/image_crop0/width", width);

  // Crop Callback
  pharos_image_transport::View crop0(nh, "/front_cam/image_crop0");
  pharos_image_transport::View crop1(nh, "/front_cam/image_crop1");
  pharos_image_transport::View crop2(nh, "/front_cam/image_crop2");
  pharos_image_transport::View crop3(nh, "/front_cam/image_crop3");
  pharos_image_transport::View crop4(nh, "/front_cam/image_crop4");
  pharos_image_transport::View crop5(nh, "/front_cam/image_crop5");
  pharos_image_transport::View crop6(nh, "/front_cam/image_crop6");
  pharos_image_transport::View crop7(nh, "/front_cam/image_crop7");
  pharos_image_transport::View crop8(nh, "/front_cam/image_crop8");
  pharos_image_transport::View crop9(nh, "/front_cam/image_crop9");
  pharos_image_transport::View crop10(nh, "/front_cam/image_crop10");
  pharos_image_transport::View crop11(nh, "/front_cam/image_crop11");
  pharos_image_transport::View crop12(nh, "/front_cam/image_crop12");
  pharos_image_transport::View crop13(nh, "/front_cam/image_crop13");
  pharos_image_transport::View crop14(nh, "/front_cam/image_crop14");
  pharos_image_transport::View crop15(nh, "/front_cam/image_crop15");
  pharos_image_transport::View crop16(nh, "/front_cam/image_crop16");
  pharos_image_transport::View crop17(nh, "/front_cam/image_crop17");
  pharos_image_transport::View crop18(nh, "/front_cam/image_crop18");
  pharos_image_transport::View crop19(nh, "/front_cam/image_crop19");


  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    if(crop0.init_&&
      crop1.init_&&
      crop2.init_&&
      crop3.init_&&
      crop4.init_&&
      crop4.init_&&
      crop5.init_&&
      crop6.init_&&
      crop7.init_&&
      crop8.init_&&
      crop9.init_&&
      crop10.init_&&
      crop11.init_&&
      crop12.init_&&
      crop13.init_&&
      crop14.init_&&
      crop15.init_&&
      crop16.init_&&
      crop17.init_&&
      crop18.init_&&
      crop19.init_){
      cv::Mat mergeImg;
      cv::Mat mergeImg_ = crop0.mat_img;
      cv::hconcat(mergeImg_, crop1.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop2.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop3.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop4.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop5.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop6.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop7.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop8.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop9.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop10.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop11.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop12.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop13.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop14.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop15.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop16.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop17.mat_img, mergeImg);
      cv::hconcat(mergeImg, crop18.mat_img, mergeImg_);
      cv::hconcat(mergeImg_, crop19.mat_img, mergeImg);


      sensor_msgs::ImagePtr imgPtr;
      imgPtr = cv_bridge::CvImage(std_msgs::Header(), crop0.cv_ptr->encoding, mergeImg).toImageMsg();

      pub.publish(imgPtr);
      // std::cout<<"pub\n";

      cv::resize( mergeImg, mergeImg_, cv::Size( 5400, 1080 ), CV_INTER_NN );
      cv::imshow("merge_image", mergeImg_);
      cv::waitKey(1);
      cv::resize( mergeImg_, mergeImg, cv::Size( 1920, 384 ), CV_INTER_NN );
      cv::imshow("merge_image_", mergeImg);
      cv::waitKey(1);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}  // end main()
