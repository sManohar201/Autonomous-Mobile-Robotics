#ifndef SENSOR_FUSION_FILTERBASE_H
#define SENSOR_FUSION_FILTERBASE_H

#include "sensor_fusion/filter_common.h"
#include "sensor_fusion/filter_utilities.h"

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <map>
#include <ostream>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace SensorFusion {

  /**
   * @brief Structure used for storing and comparing measurements
   * for priority queues. TODO: how exactly?
   *
   * Measurement units are assumed to be in meters and radians. times
   * are real-valued and measured in seconds. 
   */
  struct Measurement {
    // the time stamp of the most recent control term (needed for lagged data)
    double latestControlTime_;
    // the Mahalanobis distance threshold in number of sigmas
    double mahalanobisThresh_;
    // the real-valued time, in seconds, since some epoch
    // (presumably the start of execution, but any will do)
    double time_;
    // the topic name for this measurement. Needed
    // for capturing previous state values for new measurements
    std::string topicName_;
    // this defines which variables within this measurement
    // actually get passed into the filter. std::vector<bool>
    // is generally frowned upon, so we use ints.
    // FIXME: why bool is frowned upon? vector represents matrix
    // in row major or col major form
    std::vector<int> updateVector_;
    // the most recent control vector (needed for lagged data)
    Eigen::VectorXd latestControl_;
    // the measurement and its associated covariance
    Eigen::VectorXd measurement_;
    Eigen::VectorXd covariance_;

    // we want earlier times to have greater priority
    bool operator()(const boost::shared_ptr<Measurement> &a, const boost::shared_ptr<Measurement> &b) {
      return (*this)(*(a.get()), *(b.get()));
    }
    bool operator()(const Measurement &a, const Measurement &b) {
      // TODO: relational operator
      return a.time_ > b.time_;
    }

    Measurement() : 
        latestControlTime_(0.0),
        mahalanobisThresh_(std::numeric_limits<double>::max()),
        time_(0.0),
        topicName_("") 
    {}
  };
  typedef boost::shared_ptr<Measurement> MeasurementPtr;

  /**
   * @brief Structure used for storing and comparing filter states
   * 
   * This structure is useful when higher-level classes need to remember filter history.
   * Measurement units are assumed to be in meters and radians.
   * times are real-valued and measured in seconds.
   * 
   */
  struct FilterState {
    // the time stamp of the most recent measurement for the filter
    double lastMeasurementTime_;
    //the time stamp of the most recent control term
    double latestControlTime_;
    // the most recent control vector
    Eigen::VectorXd latestControl_;
    // the filter state vector
    Eigen::VectorXd state_;
    // the filter error covariance matrix
    Eigen::MatrixXd estimateErrorCovariance_;
    // we want the queue to be sorted from latest to earliest timestamps.
    // TODO: why not pointer as an argument as in Measurement struct
    bool operator()(const FilterState &a, const FilterState &b) {
      // TODO: relational operator
      return a.lastMeasurementTime_ < b.lastMeasurementTime_;
    }

    FilterState() :
        lastMeasurementTime_(0.0),
        latestControlTime_(0.0)
    {}
  };
  typedef boost::shared_ptr<FilterState> FilterStatePtr;


  class FilterBase {
    public:
      /**
       * @brief Construct a new Filter Base object
       * 
       */
      FilterBase();
      /**
       * @brief Destroy the Filter Base object
       * 
       */
      virtual ~FilterBase();
      /**
       * @brief Resets filter to its unintialized state
       */
      void reset();
      /**
       * @brief Computes a dynamic process noise covariance matrix using the 
       * parameterized state
       * 
       * This allows us to, e.g., not increase the pose covariance values 
       * when the vehicle is not moving
       * 
       * @param[in] state - The STATE_SIZE state vector that is used to 
       *                generate the dynamic process noise covariance.
       * @param[in] delta - delta is the time step increment
       */
      void computeDynamicProcessNoiseCovariance(const Eigen::VectorXd &state, const double delta);
      /**
       * @brief Carries out the correct steip in the predict/update cycle. This 
       * method must be implemented by subclasses.
       * 
       * @param[in] measurement - The measurement to fuse with the state estimate
       */
      virtual void correct(const Measurement &measurement) = 0;
      /**
       * @brief Get the Control object vector currently being used
       * 
       * @return const Eigen::VectorXd& - The control vector
       */
      const Eigen::VectorXd &getControl();
      /**
       * @brief Get the time at which the control term was issued
       * 
       * @return double - the time control vector was issued
       */
      double getControlTime();
      /**
       * @brief Get the Debug object variable
       * 
       * @return true - if in debug mode, false otherwise 
       */
      bool getDebug();
      /**
       * @brief Get the Estimate Error Covariance object
       * 
       * @return const Eigen::MatrixXd& 
       */
      const Eigen::MatrixXd& getEstimateErrorCovariance();
      /**
       * @brief Get the filter's Initialized Status 
       * 
       * @return true - A copy of the estimate error covariance matrix 
       */
      bool getInitializedStatus();
      /**
       * @brief Get the most recent Measurement Time 
       * 
       * @return double - The time at which we last received a measurement. 
       */
      double getLastMeasurementTime();
      /**
       * @brief Get the filter's Predicted State,
       * the state estimate before correct() is called. 
       * 
       * @return const Eigen::VectorXd& - Constant reference to the 
       * predicted state. 
       */
      const Eigen::VectorXd& getPredictedState();
      /**
       * @brief Get the filter's Process Noise Covariance
       * 
       * @return const Eigen::MatrixXd& - const reference to the process 
       * noise covariance.
       */
      const Eigen::MatrixXd& getProcessNoiseCovariance();
      /**
       * @brief Get the Sensor Timeout value (in seconds) 
       * 
       * @return double - the sensor timeout value 
       */
      double getSensorTimeout();
      /**
       * @brief Get the filter State. 
       * 
       * @return const Eigen::VectorXd&  - a const reference to the 
       * current state.
       */
      const Eigen::VectorXd& getState();
      /**
       * @brief Carries out the predict step in the predict/correct cycle.
       * Projects the state and error matrices forward using a model of 
       * the vehicle's motion. This method must be implemented by subclasses.
       * 
       * @param[in] referenceTime - The time at which the prediction is being made.
       * @param[in] delta  - the time step over which to predict.
       */
      virtual void predict(const double referenceTime, const double delta) = 0;
      /**
       * @brief Does some final preprocessing, carries out the predict/update cycle
       * 
       * @param[in] measurement - The measurement object to fuse into the filter.
       */
      virtual void processMeasurement(const Measurement &measurement);
      /**
       * @brief Sets the recent control term
       * 
       * @param control - The control term to be applied
       * @param controlTime - The time at which the control in question was received
       */
      void setControl(const Eigen::VectorXd &control, const double controlTime);
      /**
       * @brief Sets the control update vector and acceleration limits
       * 
       * @param[in] updateVector - The values the control term affects.
       * @param[in] controlTimeout - Timeout value, in seconds, after which a control is considered
       * @param[in] accelerationLimits - The acceleration limits for the control variables
       * @param[in] accelerationGains - Gains applied to the control term-derived acceleration.
       * @param[in] decelerationLimits - The deceleration limits for the control variables
       * @param[in] decelerationGains - Gains applied to the control term-derived decelerations.
       */
      // TODO: Why do you need acceleration and deceleration gains?
      void setControlParams(const std::vector<int> &updateVector, const double controlTimeout, 
              const std::vector<double> &accelerationLimits, const std::vector<double> &accelerationGains,
              const std::vector<double> &decelerationLimits, const std::vector<double> &decelerationGains);
      /**
       * @brief Set the filter into debug mode.
       *  NOTE: this will generate a lot of debug output to the provided stream.
       * The value must be a pointer to a valid ostream oubject. 
       * @param[in] debug - Whether or not to place the filter in debug mode.
       * @param[in] outStream - If debug is true, then this must have a valid pointer.
       * If the pointer is invalid, the filter will not enter debug mode. If debug is false, outStream
       * is ignored.
       */
      void setDebug(const bool debug, std::ostream *outStream = nullptr);
      /**
       * @brief enable Dynamic Process Noise Covariance  calculation
       * 
       * @param[in] dynamicProcessNoiseCovariance - whether or not to compute dynamic process noise covariance matrices.
       */
      void setUseDynamicProcessNoiseCovariance(const bool dynamicProcessNoiseCovariance);
      /**
       * @brief Set the Estimate Error Covariance 
       * 
       * @param[in] estimateErrorCovariance - the state to set as the filter's current state. 
       */
      void setEstimateErrorCovariance(const Eigen::MatrixXd &estimateErrorCovariance);
      /**
       * @brief Set the Last Measurement Time 
       * 
       * @param[in] lastMeasurementTime - the last measurement time of the filter
       */
      void setLastMeasurementTime(const double lastMeasurementTime);
      /**
       * @brief Set the Process Noise Covariance
       * This enables external initialization, which is important, as this 
       * matrix can be difficult to tune for a given implementation.
       * 
       * @param[in] processNoiseCovariance - The STATE_SIZE X STATE_SIZE process noise covariance matrix
       * to use for the filter.
       */
      void setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance);
      /**
       * @brief Set the Sensor Timeout.
       * 
       * @param[in] sensorTimeout - The time for a sensor mesurement, in seconds, to be considered having timed out.
       */
      void setSensorTimeout(const double sensorTimeout);
      /**
       * @brief Set the filter's State manually
       * 
       * @param[in] state - The state to set as the filter's current state
       */
      void setState(const Eigen::VectorXd &state);
      /**
       * @brief Ensures a given time delta is valid (helps with bag file playback issues)
       * 
       * @param delta - The time delta, in seconds, to validate.
       */
      void validateDelta(double &delta);

    protected:
      /**
       * @brief Method for setting bounds on acceleration values derived from controls
       * 
       * @param[in] state - the current state variable (e.g., linear X velocity).
       * @param[in] control - the current control commanded velocity corresponding to the state variable.
       * @param[in] accelerationLimit - limit for acceleration (regardless of driving directions).
       * @param[in] accelerationGain - gain applied to acceleration control error.
       * @param[in] decelerationLimit - limit for deceleration (moving towards zero, regardless of driving direction).
       * @param[in] decelerationGain - gain applied to deceleration control error 
       * @return double - a usuable acceleration estimate for the control vector.
       */
      inline double computeControlAcceleration(const double state, const double control, const double accelerationLimit, 
              const double accelerationGain, const double decelerationLimit, const double decelerationGain) {
        FB_DEBUG("---------- FilterBase::computeControlAcceleration -------------\n");
        // TODO: change this to two if conditionals to check if you need to accelerate or decelerate the robot.
        const double error = control - state;
        const bool sameSign = (::fabs(error) <= ::fabs(control) + 0.01);
        const double setPoint = (sameSign ? control : 0.0);
        const bool decelerating = ::fabs(setPoint) < ::fabs(state);
        double limit = accelerationLimit;
        double gain = accelerationGain;

        if (decelerating) {
          limit = decelerationLimit;
          gain = decelerationGain;
        }

        const double finalAcceleration = std::min(std::max(gain*error, -limit), limit);
        
        FB_DEBUG("Control value: " << control << "\n" <<
               "State value: " << state << "\n" <<
               "Error: " << error << "\n" <<
               "Same sign: " << (sameSign ? "true" : "false") << "\n" <<
               "Set point: " << setPoint << "\n" <<
               "Decelerating: " << (decelerating ? "true" : "false") << "\n" <<
               "Limit: " << limit << "\n" <<
               "Gain: " << gain << "\n" <<
               "Final is " << finalAcceleration << "\n");
        
        return finalAcceleration;
      }

      /**
       * @brief Converts the control term to an acceleration to be applied in the prediction step
       *        * 
       * @param[in] referenceTime - the time of the update (measurement used in the prediction step)
       * @param[in] predictionDelta - the amount of time over which we are carrying out our prediction.
       */
      void prepareControl(const double referenceTime, const double predictionDelta);

      /**
       * @brief Keeps the state euler angles in the range [-pi, pi].
       */
      virtual void wrapStateAngles();

      /**
       * @brief Tests if innovation is within N-sigmas of covariance. 
       * 
       * @param[in] innovation - the difference between the measurement and the state
       * @param[in] invCovariance - the innovation error
       * @param[in] nsigmas - number of standard deviations that are considered acceptable
       * @return  - Returns true if passed the test.
       */
      virtual bool checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                                            const Eigen::MatrixXd &invCovariance,
                                            const double nsigmas);
      /**
       * @brief Whether or not we've received any measurements.
       */
      // TODO: check for what measurements does it look for?
      bool initialized_;
      /**
       * @brief Whether or not we apply the control term
       */
      bool useControl_;
      /**
       * @brief If true, uses the robot's vehicle state and the static process
       * noise covariance matrix to generate a dynamic process noise covariance
       * matrix. 
       */
      // TODO: learn more about dynamic process noise covariance
      bool useDynamicProcessNoiseCovariance_;
      /**
       * @brief Tracks the time the filter was last updated using a measurement.
       *  This value is used to monitor sensor readings with respect to the sensorTimeout_. 
       * We also use it to compute the time delta values for our prediction step. 
       */
      double lastMeasurementTime_;
      /**
       * @brief The time of reception of the most recent control term.
       */
      double latestControlTime_;
      /**
       * @brief Timeout value, in seconds, after which a control is considered stale.
       */
      double controlTimeout_;
      /**
       * @brief the updates to the filter - both predict and correct - are driven
       * by measurements. If we get a gap in measurements for some reason, we want
       * the filter to continue estimating. When this gap occurs, as specified by this
       * timeout, we will continue to call predict() at the filter's frequency.
       */
      double sensorTimeout_;
      /**
       * @brief Which control variables are being used (e.g., not every vehicle
       * is controllable in Y or Z)
       */
      // TODO: e.g., {0, 0, 1} - {x, y, z} --> z is controllable velocities ?
      std::vector<int> controlUpdateVector_;
      /**
       * @brief Gains applied to acceleration derived from control term.
       */
      std::vector<double> accelerationGains_;
      /**
       * @brief Caps the acceleration we apply from control input.
       */
      std::vector<double> accelerationLimits_;
      /**
       * @brief Gains applied to deceleration derived from control term.
       */
      std::vector<double> decelerationGains_;
      /**
       * @brief Caps the deceleration we apply from control input
       */
      std::vector<double> decelerationLimits_;
      /**
       * @brief Variable that gets updated every time we process a measurement
       *  and we have a valid control.
       */
      Eigen::VectorXd controlAcceleration_;
      /**
       * @brief latest control term.
       */
      Eigen::VectorXd latestControl_;
      /**
       * @brief Holds the last predicted state of the filter
       */
      Eigen::VectorXd predictedState_;
      /**
       * @brief This is the robot's state vector, which is what we are trying to
       * filter. the values in this vector are what get reported by the node.
       */
      Eigen::VectorXd state_;
      /**
       * @brief Covariance matrices can be incredibly unstable. We can add a
       * small value to it at each iteration to help maintain its positive-definite 
       * property.
       */
      // TODO: read more about maintaining +ve definiteness.
      Eigen::MatrixXd covarianceEpsilon_;
      /**
       * @brief Gets updated when useDynamicProcessNoise_ is true.
       */
      Eigen::MatrixXd dynamicProcessNoiseCovariance_;
      /**
       * @brief This matrix stores the total error in our position estimate
       * (the state_variable)
       */
      Eigen::MatrixXd estimateErrorCovariance_;
      /**
       * @brief We need the identity for a few operation. Better to store it.
       */
      Eigen::MatrixXd identity_;
      /**
       * @brief As we move through the world, we follow a predict/update cycle.
       * If one were to imagine a scenario where all we did was make predictions without
       * correcting, the error in our position estimate would grow without bound.
       * This error is stored in the stateEstimateCovariance_ matrix. However, this matrix
       * doesn't answer the question of "how much" our error should grow for each time step.
       * That's where the processNoiseCovariance matrix comes in. When we make a prediction
       * using the transfer function, we add this matrix (times deltaT) to the state estimate
       * covariance matrix.
       */
      // TODO: but why?
      Eigen::MatrixXd processNoiseCovariance_;
      /**
       * @brief The kalman filter transfer function.
       *  Kalman filters and extended kalman filter project the current state forward
       * in time. This is the predit part of the predict/correct cycle. A kalman filter
       * has a typically constant matrix A that defines how to turn the current state, x, 
       * into the predicted next state. For an EKF, this matrix becomes a function f(x). 
       * However, this function can still be expressed as a matrix to make the math a little
       * cleaner. Technically  each row in the matrix is actually a function. Some rows
       * will contain many trigonometric functions, which are of course non-linear. In any
       * case, you can think of this as the 'A' matrix in the kalman filter formulation. 
       */
      Eigen::MatrixXd transferFunction_;
      /**
       * @brief The kalman filter transfer function jacobian.
       *  The transfer function is allowed to be non-linear in a EKF, but
       *  for propagating (prediction) the covariance matrix. we need to linearize
       * it about the current mean (i.e. state). this is done via a jacodbian, which
       * calculates partial derivatives of each row of the transfer function
       * matrix with respect to each state variable.
       */
      Eigen::MatrixXd transferFunctionJacobian_;
      /**
       * @brief used for outputting debug messages
       */
      std::ostream *debugStream_;
    private:
      /**
       * @brief Whether or not the filter is in debug mode.
       */
      bool debug_;
  };
} // namespace SensorFusion

#endif
