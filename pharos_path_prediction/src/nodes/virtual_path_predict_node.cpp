#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <pharos_msgs/RacingWheelStamped.h>

#include <path_predict.h>

// ros::Publisher pub;
ros::Publisher pub_sRL;
ros::Publisher pub_sRR;

geometry_msgs::Pose vehicleOdom;
bool vehicleOdomInit = false;

float predict_time_;
float steering_gain_;
float track_;
float speed_;

geometry_msgs::PoseStamped BicycleModel(geometry_msgs::PoseStamped pose_in, float vel, float alpha)
{
  geometry_msgs::PoseStamped pose_out;
  geometry_msgs::Vector3 rpy;

  tf::Quaternion xyzw;
  quaternionMsgToTF(pose_in.pose.orientation, xyzw);
  tf::Matrix3x3(xyzw).getRPY(rpy.x, rpy.y, rpy.z);
// quaternionTFToMsg(quat_tf , quat_msg);

  float wheelBase = 2.65;
  float d = vel * 0.1;
  float beta = d / wheelBase * tan(alpha);
  float theta = rpy.z + beta;
  pose_out.pose.position.x = pose_in.pose.position.x + d * cos(theta);
  pose_out.pose.position.y = pose_in.pose.position.y + d * sin(theta);
  pose_out.pose.position.z = pose_in.pose.position.z;

  pose_out.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rpy.x, rpy.y, theta);
  return pose_out;
}

geometry_msgs::PoseStamped shiftingPath(geometry_msgs::PoseStamped pose_in, float offset)
{
  geometry_msgs::PoseStamped pose_out = pose_in;
  geometry_msgs::Vector3 rpy;
  tf::Quaternion xyzw;
  quaternionMsgToTF(pose_out.pose.orientation, xyzw);
  tf::Matrix3x3(xyzw).getRPY(rpy.x, rpy.y, rpy.z);

  pose_out.pose.position.x += offset*cos(rpy.z+M_PI/2);
  pose_out.pose.position.y += offset*sin(rpy.z+M_PI/2);

  return pose_out;
}

void SpeedCallback(const std_msgs::Float32::ConstPtr msg)
{
  speed_ = msg->data;
}

void RacingWheelCallback(const pharos_msgs::RacingWheelStamped::ConstPtr msg)
{
  // if( !vehicleOdomInit ) return;
  // float vel = 10. / predict_time_;
  float vel = speed_;
  float alpha = msg->state.steering * M_PI/180.0 * steering_gain_/15;

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1;

  ros::Duration d(0.1);
  ros::Time t = ros::Time::now();
  path.header.stamp = t;
  path.header.frame_id = "virtual_vehicle";

  nav_msgs::Path path_RL = path;
  nav_msgs::Path path_RR = path;

  for(int i=1; i<=predict_time_*10; i++)
  {
    pose = BicycleModel(pose, vel, alpha);
    pose.header.seq = i;
    pose.header.stamp = t+d*i;
    // pose.header.frame_id = "odom";
    path.poses.push_back(pose);

    path_RL.poses.push_back(shiftingPath(pose, track_/2));
    path_RR.poses.push_back(shiftingPath(pose,-track_/2));
  }
  // pub.publish(path);
  pub_sRL.publish(path_RL);
  pub_sRR.publish(path_RR);
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "path");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<float>("predict_time", predict_time_, 5);
  pnh.param<float>("steering_gain", steering_gain_, 1.0);
  pnh.param<float>("track", track_, 1.6);
  
  std::string master_interface;
  pnh.param<std::string>("master_interface", master_interface, "/t300/state");

  ros::Subscriber sub_speed = nh.subscribe("/virtual/virtual_vehicle/speedometer", 1, SpeedCallback);
  ros::Subscriber sub_master_interface = nh.subscribe(master_interface, 1, RacingWheelCallback);
  // ros::Subscriber sub2 = nh.subscribe("/odom/vehicle", 1, OdomVehicleCallback, ros::TransportHints().udp());

  // pub = nh.advertise<nav_msgs::Path>(std::string("/path/predicted"), 1);
  pub_sRL = nh.advertise<nav_msgs::Path>(std::string("/virtual/virtual_vehicle/predicted_path/RL"), 1);
  pub_sRR = nh.advertise<nav_msgs::Path>(std::string("/virtual/virtual_vehicle/predicted_path/RR"), 1);

  ros::spin();

  // ros::Rate loop_rate(100);
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  return 0;
}  // end main()
