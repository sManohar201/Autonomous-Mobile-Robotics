#include "sensor_fusion/filter_common.h"
#include "sensor_fusion/ekf.h"

#include <XmlRpcException.h>

#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>


namespace SensorFusion 
{

  Ekf::Ekf(std::vector<double>) : FilterBase() {}

  Ekf::~Ekf() {}

  void Ekf::correct(const Measurement &measurement) {
    FB_DEBUG("----------------------- Ekf::correct ------------------------\n" << 
             "State is:\n" << state_ << "\n"
             "Topic is:\n" << measurement.topicName_ << "\n"
             "Measurement is:\n" << measurement.measurement_ << "\n"
             "Measurement topic name is:\n" << measurement.topicName_ << "\n\n"
             "Measurement covariance is:\n" << measurement.covariance_ << "\n");
  }

} // namespace 
