#include "sensor_fusion/ros_filter_utilities.h"
#include "sensor_fusion/filter_common.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/console.h>

#include <string>
#include <vector>

std::ostream& operator<<(std::ostream& os, const tf2::Vector3 &vec) {
  os << "(" << std::setprecision(20) << vec.getX() << " " << vec.getY() << " " << vec.getZ() << ")\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const tf2::Quaternion& quat) {
  double roll, pitch, yaw;
  tf2::Matrix3x3 orTmp(quat);
  orTmp.getRPY(roll, pitch, yaw);

  os << "(" << std::setprecision(20) << roll << ", " << pitch << ", " << yaw << ")\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const tf2::Transform& trans) {
  os << "Origin: " << trans.getOrigin() << 
        "Rotation (RPY): " << trans.getRotation();

  return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<double> &vec) {
  os << "(" << std::setprecision(20);
  for (size_t i=0; i<vec.size(); i++) {
    os << vec[i] << " ";
  }
  os << ")\n";
  return os;
}

