#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "pharos_msgs/RacingWheelStamped.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_odom_;
ros::Subscriber sub_wheel_;
ros::Publisher pub_odom_;
ros::Publisher pub_transform_;

nav_msgs::Odometry ego_msg_, vv_msg_; //ego_vehicle, virtual vehicle

geometry_msgs::Pose2D pose_;

float steering_coeff_ = 15;

bool isInit_ = false;

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


void VehicleOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ego_msg_ = *msg;
    if(!isInit_){
        pose_.x = msg->pose.pose.position.x;
        pose_.y = msg->pose.pose.position.y;

        ROS_WARN("pose init to x: %f, y: %f", pose_.x, pose_.y);

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        static double roll, pitch;
        m.getRPY(roll, pitch, pose_.theta);

        isInit_ = true;
    }
}

void UpdateTF(){

    vv_msg_.pose.pose.position.x = pose_.x;
    vv_msg_.pose.pose.position.y = pose_.y;
    tf::Quaternion quat;
    double roll = 0;
    double pitch = 0;
    quat.setRPY(roll, pitch + M_PI/180*20, pose_.theta);
    tf::Transform tf_transform;
    // tf_transform.setOrigin( tf::Vector3(0,0,0) );
    tf_transform.setRotation(quat);
    vv_msg_.pose.pose.position.z = ego_msg_.pose.pose.position.z;

    vv_msg_.header = ego_msg_.header;
    vv_msg_.child_frame_id = "third_person_view";

    vv_msg_.pose.pose.orientation.x = tf_transform.getRotation().x();
    vv_msg_.pose.pose.orientation.y = tf_transform.getRotation().y();
    vv_msg_.pose.pose.orientation.z = tf_transform.getRotation().z();
    vv_msg_.pose.pose.orientation.w = tf_transform.getRotation().w();

    pub_odom_.publish(vv_msg_);

    ////////////////
    static tf::TransformBroadcaster br;
    static geometry_msgs::Transform transform;
    static geometry_msgs::TransformStamped tf_stamped_;

    // std::cout<<Odom->child_frame_id<<": "<<Odom->header.stamp<<std::endl;

    transform.translation.x = vv_msg_.pose.pose.position.x;
    transform.translation.y = vv_msg_.pose.pose.position.y;
    transform.translation.z = vv_msg_.pose.pose.position.z+2.8;
    transform.rotation = vv_msg_.pose.pose.orientation;

    tf_stamped_.header.stamp = vv_msg_.header.stamp;
    tf_stamped_.header.frame_id = vv_msg_.header.frame_id;
    tf_stamped_.child_frame_id = vv_msg_.child_frame_id;
    tf_stamped_.transform = transform;

    br.sendTransform(tf_stamped_);

    ////////////////
    geometry_msgs::Pose ego2cam;

    static tf::StampedTransform transform_ego_view;
    static tf::TransformListener listener;
    static bool is_TF_init = false;
    try{
      listener.lookupTransform("ego_vehicle", "third_person_view", ros::Time(0), transform_ego_view);
      is_TF_init = true;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

    ego2cam.position.x = transform_ego_view.getOrigin().x();
    ego2cam.position.y = transform_ego_view.getOrigin().y();
    ego2cam.position.z = transform_ego_view.getOrigin().z();

    ego2cam.orientation.x = transform_ego_view.getRotation().x();
    ego2cam.orientation.y = transform_ego_view.getRotation().y();
    ego2cam.orientation.z = transform_ego_view.getRotation().z();
    ego2cam.orientation.w = transform_ego_view.getRotation().w();

    pub_transform_.publish(ego2cam);
}

void RacingWheelCallback(const pharos_msgs::RacingWheelStamped::ConstPtr& msg)
{
    if(!isInit_) return;

    static ros::Time prev_time = ros::Time::now();
    ros::Duration dt = ros::Time::now() - prev_time;
    static double vehicle_speed = 0;
    vehicle_speed += msg->state.accel/100 * 0.1;
    vehicle_speed -= msg->state.brake/100 * 0.1;

    vehicle_speed *= 0.99;
    vehicle_speed -= 1.0/100;
    std::cout<<"vehicle_speed: "<<vehicle_speed<<std::endl;

    if(vehicle_speed > 0.1){
        //Bicycle Model
        float L = 2.65;
        double d = vehicle_speed*dt.toSec();
        double beta = d/L*tanf(msg->state.steering/steering_coeff_);

        double R = d/beta;
        double Cx = pose_.x-sin(pose_.theta)*R;
        double Cy = pose_.y+cos(pose_.theta)*R;
        ROS_WARN("log: %f %f %f %f", pose_.x, d/beta, Cx, Cy);
        pose_.theta += beta;
        pose_.x = Cx+sin(pose_.theta)*R;
        pose_.y = Cy-cos(pose_.theta)*R;
    }

    UpdateTF();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    sub_odom_ = nh.subscribe("/carla/ego_vehicle/odometry", 10, VehicleOdomCallback); //topic que function
    sub_wheel_ = nh.subscribe("/t300/state", 10, RacingWheelCallback); //topic que function
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/carla/virtual_vehicle", 10); //topic que
    pub_transform_ = nh.advertise<geometry_msgs::Pose>("/carla/ego_vehicle/rgb_view/control/set_transform", 10); //topic que

    ros::spin();

    return 0;
}