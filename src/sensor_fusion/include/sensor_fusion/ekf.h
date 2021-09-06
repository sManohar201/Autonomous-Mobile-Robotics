#ifdef SENSOR_FUSION_EKF_H 
#define SENSOR_FUSION_EKF_H 

#include "robot_localization/filter_base.h"

#include <fstream>
#include <vector>
#include <set>
#include <queue>


namespace SensorFusion 
{
  /**
   * @brief Externded kalman filter class
   * 
   * Implementation of an extended kalman filter (EKF). This class derives 
   * from FilterBase and overrides the predict() and correct() methods in 
   * keeping with the discrete time EKF algorithm.
   */

  // TODO: discrete time EKF algorithm
  class Ekf : public FilterBase 
  {
    public:
      /**
       * @brief Construct a new Ekf object
       * 
       * @param args - Generic argument container (not used here, but 
       * needed so that the ROS filters can pass arbitrary arguments 
       * to templated filter types).
       */
      explicit Ekf(std::vector<double> args = std::vetor<double>());

      /**
       * @brief Destroy the Ekf object
       * 
       */
      ~Ekf();

      /**
       * @brief Carries out the correct step in the predict/update cycle.
       * 
       * @param measurement - the measurement to fuse with our estimate.
       */
      void correct(const Measurement &measurement);

      /**
       * @brief Carries out the predict step in the predict/update cycle.
       * 
       * Projects the state and error matrices forward using a model of 
       * the vehicle's motion.
       * 
       * @param referenceTime - The time at which the prediction is being
       * made.
       * @param delta - The time step over which to predict.
       */
      void predict(const double referenceTime, const double delta);

  }  
} // namespace SensorFusion 

#endif