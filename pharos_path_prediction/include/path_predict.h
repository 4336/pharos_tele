#ifndef PHAROS_PATH_PREDICTION_H
#define PHAROS_PATH_PREDICTION_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Path.h>
#include <pharos_msgs/CAN_GWAY_header.h>

namespace pharos_path_prediction
{
class Prediction
{
 public:
  explicit Prediction(ros::NodeHandle nh);

  nav_msgs::Path path;

 private:
  bool init_;

};
}

#endif  // NODE_EXAMPLE_TALKER_H
