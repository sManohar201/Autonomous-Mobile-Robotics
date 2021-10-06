#include "sensor_fusion/ros_filter_types.h"

#include <ros/ros.h>

#include <cstdlib>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ukf_navigation_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  std::vector<double> args(3, 0);

  nhLocal.param("alpha", args[0], 0.001);
  nhLocal.param("kappa", args[1], 0.0);
  nhLocal.param("beta",  args[2], 2.0);

  SensorFusion::RosUkf ukf(nh, nhLocal, args);
  ukf.initialize();
  ros::spin();

  return EXIT_SUCCESS;
}
