#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "ackermann_msgs/AckermannDrive.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_virtual_odom_, sub_remote_odom_;
ros::Publisher pub_goal_, pub_target_speed_; // pub_cmd_;

nav_msgs::Odometry vv_odom_, rv_odom_;

geometry_msgs::PoseStamped goal_;

std_msgs::Float64 target_speed_;

float max_speed_ = 0;


// ackermann_msgs::AckermannDrive cmd_msg_;

void VirtualVehicleCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vv_odom_ = *msg;
}

void RemoteVehicleCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	goal_.header = vv_odom_.header;
	goal_.pose = vv_odom_.pose.pose;
	pub_goal_.publish(goal_);

	target_speed_.data = max_speed_/3.6;
	pub_target_speed_.publish(target_speed_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_virtual_vehicle_control");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	std::string param;

	pnh.param<float>("max_speed", max_speed_, 10);

	bool latency_on;
	pnh.param<bool>("latency_on", latency_on, true);
	if(latency_on){
		sub_remote_odom_ = nh.subscribe("/carla/ego_vehicle/odometry", 10, RemoteVehicleCallback); //topic que function

		sub_virtual_odom_ = nh.subscribe("/virtual/virtual_vehicle/odometry", 10, VirtualVehicleCallback); //topic que function

	}else{
		sub_remote_odom_ = nh.subscribe("/carla/ego_vehicle/odometry", 10, RemoteVehicleCallback); //topic que function

		sub_virtual_odom_ = nh.subscribe("/virtual/virtual_vehicle/odometry", 10, VirtualVehicleCallback); //topic que function

	}

	// pub_cmd_ = nh.advertise<ackermann_msgs::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 10); //topic que

	pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/carla/ego_vehicle/goal", 10); //topic que
	pub_target_speed_ = nh.advertise<std_msgs::Float64>("/carla/ego_vehicle/target_speed", 10); //topic que

	ros::spin();

	return 0;
}