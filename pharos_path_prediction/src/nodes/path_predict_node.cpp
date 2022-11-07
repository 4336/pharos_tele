#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pharos_msgs/CAN_GWAY_header.h>
#include <pharos_msgs/SCBADUheader.h>
#include <pharos_msgs/StateStamped2016.h>

#include <path_predict.h>

// ros::Publisher pub;
ros::Publisher pub_rRL;
ros::Publisher pub_rRR;
ros::Publisher pub_sRL;
ros::Publisher pub_sRR;

pharos_msgs::CAN_GWAY_header CAN;
geometry_msgs::Pose vehicleOdom;
bool vehicleOdomInit = false;

float predict_time_;
float steering_gain_;
float track_;

// void OdomVehicleCallback(const nav_msgs::OdometryConstPtr &msg)
// {
//   vehicleOdom = msg->pose.pose;
//   vehicleOdomInit = true;
// }

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

void CANGwayCallback(const pharos_msgs::CAN_GWAY_header::ConstPtr veh)
{
  // if( !vehicleOdomInit ) return;
  float vel = (veh->GWAY1.Wheel_Velocity_FL+veh->GWAY1.Wheel_Velocity_FR+veh->GWAY1.Wheel_Velocity_RL+veh->GWAY1.Wheel_Velocity_RR)/4/3.6;
  vel += 0.1;
  float alpha = veh->GWAY2.Steering_Angle * M_PI/180.0 * steering_gain_/13.4;

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1;

  ros::Duration d(0.1);
  ros::Time t = ros::Time::now();
  path.header.stamp = t;
  path.header.frame_id = "vehicle_frame";

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
  pub_rRL.publish(path_RL);
  pub_rRR.publish(path_RR);
}

void OldCANGwayCallback(const pharos_msgs::StateStamped2016::ConstPtr veh)
{
  // if( !vehicleOdomInit ) return;
  float vel = (veh->state.rear_left_wheel+veh->state.rear_right_wheel)/2/3.6;
  vel += 0.1;
  float alpha = veh->state.wheel_angle;// * M_PI/180.0 * steering_gain_/13.4;

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1;

  ros::Duration d(0.1);
  ros::Time t = veh->header.stamp;
  path.header.stamp = t;
  path.header.frame_id = "vehicle_frame";

  nav_msgs::Path path_RL = path;
  nav_msgs::Path path_RR = path;
std::cout<<alpha<<std::endl;
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
  pub_rRL.publish(path_RL);
  pub_rRR.publish(path_RR);
}

void SCUBADUCallback(const pharos_msgs::SCBADUheader::ConstPtr msg)
{
  // if( !vehicleOdomInit ) return;
  float vel = 10. / predict_time_;
  float alpha = msg->scbadu.steering * M_PI/180.0 * steering_gain_/13.4;

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1;

  ros::Duration d(0.1);
  ros::Time t = ros::Time::now();
  path.header.stamp = t;
  path.header.frame_id = "vehicle_frame";

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

  ros::Subscriber sub_rover = nh.subscribe("/CAN_Gateway", 1, CANGwayCallback);
  ros::Subscriber sub_rover_old = nh.subscribe("/vehicle/state2016", 1, OldCANGwayCallback);
  ros::Subscriber sub_station = nh.subscribe("/master/interface", 1, SCUBADUCallback);
  // ros::Subscriber sub2 = nh.subscribe("/odom/vehicle", 1, OdomVehicleCallback, ros::TransportHints().udp());

  // pub = nh.advertise<nav_msgs::Path>(std::string("/path/predicted"), 1);
  pub_rRL = nh.advertise<nav_msgs::Path>(std::string("/path/roverRL"), 1);
  pub_rRR = nh.advertise<nav_msgs::Path>(std::string("/path/roverRR"), 1);

  pub_sRL = nh.advertise<nav_msgs::Path>(std::string("/path/stationRL"), 1);
  pub_sRR = nh.advertise<nav_msgs::Path>(std::string("/path/stationRR"), 1);

  ros::spin();

  // ros::Rate loop_rate(100);
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  return 0;
}  // end main()
