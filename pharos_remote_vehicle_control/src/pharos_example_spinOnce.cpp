#include "ros/ros.h"
#include "pharos_example/PharosExampleStamped.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub;
ros::Publisher pub;

pharos_example::PharosExampleStamped msg_;

double spin_dt_ = 0;

void Callback(const pharos_example::PharosExampleStamped::ConstPtr& msg)
{
	msg_=*msg;
	std::cout<<msg_<<std::endl;
}

void Publisher()
{
	pub.publish(msg_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	int spin_rate;
	pnh.param("publish_rate", spin_rate, 100);
	spin_dt_ = 1./spin_rate;

	sub = nh.subscribe("test_sub", 10, Callback); //topic que function
	pub = nh.advertise<pharos_example::PharosExampleStamped>("test_pub", 10); //topic que

	ros::Rate loop_rate(spin_rate);
	int count = 0;
	while (ros::ok())
	{
		Publisher();

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}