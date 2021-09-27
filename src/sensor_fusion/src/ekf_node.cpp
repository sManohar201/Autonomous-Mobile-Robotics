#include "sensor_fusion/ros_filter_types.h"

#include <cstdlib>
#include <ros/ros.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "ekf_fusion_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  SensorFusion::RosEkf ekf(nh,nh_priv);
  ekf.initialize();
  ros::spin();

  return EXIT_SUCCESS;
}