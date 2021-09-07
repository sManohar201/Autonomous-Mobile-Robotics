#ifndef SENSOR_FUSION_FILTER_UTILITIES_H
#define SENSOR_FUSION_FILTER_UTILITIES_H

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// FIXME: define this where it is appropriate
#define FB_DEBUG(msg) if (getDebug()) { *debugStream_ << msg; }

// Handy methods for debug output
std::ostream& operator<<(std::ostream& os, const Eigen::MatrixXd &mat);
std::ostream& operator<<(std::ostream& os, const Eigen::VectorXd &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<size_t> &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<int> &vec);  // TODO: why size_t and int?

namespace SensorFusion {
  namespace FilterUtilities {

    /**
     * @brief Utility method keeping RPY angles in the range [-pi, +pi]
     * 
     * @param[in] rotation - The rotaion to bind
     * @return double - The bounded value
     */
    double clampRotation(double rotation);

    /**
     * @brief Utility method for appending tf2 prefixes cleanly
     * 
     * @param[in] tfPrefix - the tf2 prefix to append
     * @param[in, out] framId - the resulting frame_id value
     */
    // TODO: check for this usage in the future.
    void appendPrefix(std::string tfPrefix, std::string &framId);

  } // namespace FilterUtilities

} // namespace SensorFusion

#endif
