#ifndef SENSOR_FUSION_ROS_FILTER_TYPES_H
#define SENSOR_FUSION_ROS_FILTER_TYPES_H 


#include "sensor_fusion/ros_filter.h"
#include "sensor_fusion/ekf.h"

namespace SensorFusion
{

  typedef RosFilter<Ekf> RosEkf;

}

#endif // SENSOR_FUSION_ROS_FILTER_TYPES_H 
 