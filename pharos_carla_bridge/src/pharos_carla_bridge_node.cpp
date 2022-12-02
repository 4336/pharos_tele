#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "pharos_msgs/SCBADUheader.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub;
ros::Publisher pub;

void Callback(const pharos_msgs::SCBADUheader::ConstPtr& msg)
{
	carla_msgs::CarlaEgoVehicleControl cmd;
	cmd.header.stamp = msg->header.stamp;
	cmd.header.frame_id = "t300";

	cmd.throttle = msg->scbadu.accel / 100;
	cmd.steer =-msg->scbadu.steering / 360;
	if(cmd.steer > 1) cmd.steer = 100;
	if(cmd.steer <-1) cmd.steer =-100;
	cmd.brake = msg->scbadu.brake / 100;

	if(msg->scbadu.clutch > 1){
		cmd.reverse = true;
		cmd.gear = -1;
		cmd.throttle = msg->scbadu.clutch / 100;
	}

	pub.publish(cmd);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string input_topic;
	pnh.param<std::string>("input_topic", input_topic,"/t300/state");

	std::string output_topic;
	pnh.param<std::string>("output_topic", output_topic, "/carla/ego_vehicle/vehicle_control_cmd_manual");

	sub = nh.subscribe(input_topic, 10, Callback); //topic que function
	pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>(output_topic, 10); //topic que

	ros::spin();

	return 0;
}