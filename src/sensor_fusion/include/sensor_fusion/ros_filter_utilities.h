#ifndef SENSOR_FUSION_ROS_FILTER_UTILITIES_H
#define SENSOR_FUSION_ROS_FILTER_UTILITIES_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#define RF_DEBUG(msg) if (filter_.getDebug()) { debugStream_ << msg; }

std::ostream& operator<<(std::ostream& os, const tf2::Vector3 &vec);
std::ostream& operator<<(std::ostream& os, const tf2::Quaternion &quat);
std::ostream& operator<<(std::ostream& os, const tf2::Transform &trans);
std::ostream& operator<<(std::ostream& os, const std::vector<double> &vec);


namespace SensorFusion {

  namespace RosFilterUtilities {

      /**
       * @brief Get the Yaw angle from the quaternion
       * 
       * @param[in] quat - angles in quaternion
       * @return double - return yaw angle from the input quaternion 
       */
      double getYaw(const tf2::Quaternion quat);

      /**
       * @brief Method for safely obtaining transforms.
       * 
       * @param[in] buffer - tf buffer object to use for looking up the transform.
       * @param[in] targetFrame - the target frame of the desired transform.
       * @param[in] sourceFrame - the source frame of the desired transform.
       * @param[in] time - the time at which we want the transform.
       * @param[in] timeout - how long to block before falling back to last transform.
       * @param[out] targetFrameTrans - the resulting transform object.
       * @param[in] silent - whether or not to print transform warnings
       * @return - sets the values of @p targetFrameTrans and returns true if successful,  
       * false otherwise.
       * 
       * This method attempts to obtain a transform from the @p sourceFrame to 
       * the @p targetFRame at the specific @p time. If no transform is available 
       * at the time,it attempts to simply ojbtain the latest transform. If that still 
       * fails, then the method checks to see if the transform is going from a given 
       * frame_id to itself. If any of these checks succeed, the method sets the value of 
       * @p targetFrameTrans and returns true, otherwise it returns false.
       */
      bool lookupTransformSafe(const tf2_ros::Buffer &buffer, const std::string &targetFrame,
                               const std::string &sourceFrame, const ros::Time &time,
                               const ros::Duration &timeout, tf2::Transform &targetFrameTrans,
                               const bool silent = false);
  } // namespace RosFilterUtilities

} // namespace SensorFusion

#endif // SENSOR_FUSION_ROS_FILTER_UTILITIES_H 