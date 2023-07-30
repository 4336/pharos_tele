#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "pharos_msgs/RacingWheelStamped.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_control_mode_;
ros::Subscriber sub_odom_;
ros::Subscriber sub_wheel_;
ros::Publisher pub_odom_;
ros::Publisher pub_speed_mps_, pub_speed_kph_;
ros::Publisher pub_transform_;

nav_msgs::Odometry ego_msg_, vv_msg_; //ego_vehicle, virtual vehicle
pharos_msgs::RacingWheelStamped wheel_msg_;
geometry_msgs::Pose2D pose_;

float steering_coeff_ = 15;

bool isEgoInit_ = false;
bool is_manual_ = false;

int spin_rate_;
float spin_dt_;

Eigen::Quaternionf rpy2quat(float roll, float pitch, float yaw){
    //Roll pitch and yaw in Radians    
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
    return q;
}

Eigen::Vector3f quat2rpy(Eigen::Quaternionf q){
    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;
    return euler;
}

void RacingWheelCallback(const pharos_msgs::RacingWheelStamped::ConstPtr& msg)
{
    wheel_msg_ = *msg;
}

void VehicleOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!isEgoInit_){

        ROS_WARN("pose init to x: %f, y: %f", pose_.x, pose_.y);

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        static double roll, pitch;
        m.getRPY(roll, pitch, pose_.theta);

        pose_.x = msg->pose.pose.position.x;
        pose_.y = msg->pose.pose.position.y;

        // float rear_axle_dist = -1.4;
        // pose_.x = msg->pose.pose.position.x + rear_axle_dist * cos(pose_.theta);
        // pose_.y = msg->pose.pose.position.y + rear_axle_dist * sin(pose_.theta);

        isEgoInit_ = true;
    }

    ego_msg_ = *msg;
}

void UpdateCamera(tf::Transform vehicle_transform){

    static tf::TransformListener listener;
    static bool isListenerInit = false;
    static ros::Time listenerInitTime = ros::Time::now();

    static bool isTransformInit = false;
    static tf::StampedTransform camera_transform;
    if(!isTransformInit){
        if(!isListenerInit) if(ros::Time::now() - listenerInitTime > ros::Duration(1)) isListenerInit = true;
        else return;

        try{
          listener.lookupTransform("virtual_vehicle", "third_person_view", ros::Time(0), camera_transform);
          isTransformInit = true;
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          isTransformInit = false;
          return;
        }
    }

    tf::Transform transform = vehicle_transform * camera_transform;

    geometry_msgs::Pose map2cam;

    map2cam.position.x = transform.getOrigin().x();
    map2cam.position.y = transform.getOrigin().y();
    map2cam.position.z = transform.getOrigin().z();

    map2cam.orientation.x = transform.getRotation().x();
    map2cam.orientation.y = transform.getRotation().y();
    map2cam.orientation.z = transform.getRotation().z();
    map2cam.orientation.w = transform.getRotation().w();

    pub_transform_.publish(map2cam);
}

void ModelUpdate()
{
    if(!isEgoInit_) return;

    // Speed model //
    static ros::Time prev_time = ros::Time::now();
    static double vehicle_speed = 0;

    vehicle_speed += wheel_msg_.state.accel*spin_dt_ * 0.1;
    vehicle_speed -= wheel_msg_.state.brake*spin_dt_ * 0.2;

    double linear_dcc = 1 - spin_dt_ * 0.1;
    vehicle_speed *= linear_dcc;

    double const_dcc = spin_dt_ * 0.1;
    if(vehicle_speed > const_dcc) vehicle_speed -= const_dcc;
    else if(fabs(vehicle_speed) > 0) vehicle_speed = 0;
    else vehicle_speed += const_dcc;

    // Steering model //
    if(vehicle_speed > 0.1){
        //Bicycle Model
        static double L = 2.65;
        double d = vehicle_speed*spin_dt_;
        std::cout<<"vehicle_speed"<<": "<<vehicle_speed;
        std::cout<<"spin_dt_"<<": "<<spin_dt_;
        double steering_angle = wheel_msg_.state.steering*M_PI/180.;
        std::cout<<"steering_angle"<<": "<<steering_angle;
        double wheel_angle = steering_angle / steering_coeff_;
        std::cout<<"wheel_angle"<<": "<<wheel_angle;
        double beta = d/L*tanf(wheel_angle);
        std::cout<<"beta"<<": "<<beta;

        double R = d/beta;
        double Cx = pose_.x-sin(pose_.theta)*R;
        double Cy = pose_.y+cos(pose_.theta)*R;
        pose_.theta += beta;
        pose_.x = Cx+sin(pose_.theta)*R;
        pose_.y = Cy-cos(pose_.theta)*R;
    }


    vv_msg_.pose.pose.position.x = pose_.x;
    vv_msg_.pose.pose.position.y = pose_.y;
    std::cout<<vv_msg_.pose.pose.position;
    tf::Quaternion quat;
    double roll = 0;
    double pitch = 0;
    quat.setRPY(roll, pitch, pose_.theta);
    tf::Transform tf_transform;
    // tf_transform.setOrigin( tf::Vector3(0,0,0) );
    tf_transform.setRotation(quat);
    vv_msg_.pose.pose.position.z = ego_msg_.pose.pose.position.z;

    vv_msg_.header.stamp = ros::Time::now();
    vv_msg_.header.frame_id = "map";
    vv_msg_.child_frame_id = "virtual_vehicle";

    vv_msg_.pose.pose.orientation.x = tf_transform.getRotation().x();
    vv_msg_.pose.pose.orientation.y = tf_transform.getRotation().y();
    vv_msg_.pose.pose.orientation.z = tf_transform.getRotation().z();
    vv_msg_.pose.pose.orientation.w = tf_transform.getRotation().w();

    pub_odom_.publish(vv_msg_);

    static std_msgs::Float32 speed; // m/s
    speed.data = vehicle_speed;
    pub_speed_mps_.publish(speed);
    speed.data *= 3.6;
    pub_speed_kph_.publish(speed);

    // Update Camera
    tf::Vector3 origin(vv_msg_.pose.pose.position.x, vv_msg_.pose.pose.position.y, vv_msg_.pose.pose.position.z);
    tf_transform.setOrigin(origin);
    UpdateCamera(tf_transform); //relative to ego_vehicle
}

void ControlModeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	is_manual_ = msg->data;
}

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ros param
    pnh.param("publish_rate", spin_rate_, 100);

    // ros sub & pub
    sub_control_mode_ = nh.subscribe("/carla/ego_vehicle/vehicle_control_manual_override", 10, ControlModeCallback); //topic que function
    sub_odom_ = nh.subscribe("/carla/ego_vehicle/odometry", 10, VehicleOdomCallback); //topic que function
    sub_wheel_ = nh.subscribe("/t300/state", 10, RacingWheelCallback); //topic que function
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/virtual/virtual_vehicle/odometry", 10); //topic que
    pub_speed_mps_ = nh.advertise<std_msgs::Float32>("/virtual/virtual_vehicle/speedometer", 10); //topic que
    pub_speed_kph_ = nh.advertise<std_msgs::Float32>("/virtual/virtual_vehicle/speedometer/kph", 10); //topic que
    pub_transform_ = nh.advertise<geometry_msgs::Pose>("/carla/virtual_cam/control/set_transform", 10); //topic que

    // ros spin
    ros::Rate loop_rate(spin_rate_);
    spin_dt_ = 1./spin_rate_;
    unsigned long seq = 0;
    while (ros::ok())
    {
        static bool reset;
        if(pnh.getParam("reset", reset)){
            isEgoInit_ = (!reset) && isEgoInit_;
            pnh.setParam("reset", false);
        }

        if(!is_manual_) ModelUpdate();

        ros::spinOnce();

        loop_rate.sleep();
        ++seq;
    }

    return 0;
}