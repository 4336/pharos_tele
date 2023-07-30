#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Bool.h"

#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_ackermann_msgs/EgoVehicleControlInfo.h"
#include "pharos_msgs/RacingWheelStamped.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_control_mode_;
ros::Subscriber sub_t300_;
ros::Subscriber sub_autonomy_;
ros::Publisher pub_control_msg_;

carla_msgs::CarlaEgoVehicleControl manual_control_msg_;
carla_msgs::CarlaEgoVehicleControl auto_control_msg_;

double spin_dt_ = 0;

bool is_manual_ = false;

void T300_Callback(const pharos_msgs::RacingWheelStamped::ConstPtr& msg)
{
	static int gear = 1;
	static pharos_msgs::RacingWheelStamped msg_prev;

	manual_control_msg_.header.stamp = msg->header.stamp;
	manual_control_msg_.header.frame_id = "t300";

	manual_control_msg_.throttle = msg->state.accel / 100;
	manual_control_msg_.steer =-msg->state.steering / 360;
	manual_control_msg_.steer = std::min(std::max((double)manual_control_msg_.steer, -1.0), 1.0);
	manual_control_msg_.brake = msg->state.brake / 100;

	if(msg->state.gear_down && msg_prev.state.gear_down == 0) gear--;
	if(msg->state.gear_up && msg_prev.state.gear_up == 0) gear++;
	gear = std::min(std::max(gear, -1), 5);

	manual_control_msg_.reverse = gear < 0;
	manual_control_msg_.gear = gear;
	manual_control_msg_.manual_gear_shift = true;

	msg_prev = *msg;
}

void Autonomy_Callback(const carla_ackermann_msgs::EgoVehicleControlInfo::ConstPtr& msg)
{
	auto_control_msg_ = msg->output;
}

void ControlModeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	is_manual_ = msg->data;
}

int main(int argc, char **argv)
{
	// ROS init //
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	std::string param;

	sub_control_mode_ = nh.subscribe("/carla/ego_vehicle/vehicle_control_manual_override", 10, ControlModeCallback); //topic que function

	pnh.param<std::string>("input_msg", param,"/t300/state");
	sub_t300_ = nh.subscribe(param, 10, T300_Callback); //topic que function

	pnh.param<std::string>("output_msg", param, "/carla/ego_vehicle/vehicle_control_cmd_manual");
	pub_control_msg_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>(param, 10); //topic que

	// ROS spin //
	int spin_rate = 1;
	pnh.param("spin_rate", spin_rate, 1);
	spin_dt_ = 1./spin_rate;

	ros::WallRate loop_rate(spin_rate);
	int count = 0;
	while (ros::ok())
	{
		pub_control_msg_.publish(manual_control_msg_);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}