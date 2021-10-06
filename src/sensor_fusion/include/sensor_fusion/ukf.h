
#ifndef SENSOR_FUSION_UKF_H
#define SENSOR_FUSION_UKF_H

#include "sensor_fusion/filter_base.h"

#include <fstream>
#include <vector>
#include <set>
#include <queue>

namespace SensorFusion 
{

//! @brief Unscented Kalman filter class
//!
//! Implementation of an unscenter Kalman filter (UKF). This
//! class derives from FilterBase and overrides the predict()
//! and correct() methods in keeping with the discrete time
//! UKF algorithm. The algorithm was derived from the UKF
//! Wikipedia article at
//! (http://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter)
//! ...and this paper:
//! J. J. LaViola, Jr., “A comparison of unscented and extended Kalman
//! filtering for estimating quaternion motion,” in Proc. American Control
//! Conf., Denver, CO, June 4–6, 2003, pp. 2435–2440
//! Obtained here: http://www.cs.ucf.edu/~jjl/pubs/laviola_acc2003.pdf
//!
class Ukf: public FilterBase
{
  public:
    //! @brief Constructor for the Ukf class
    //!
    //! @param[in] args - Generic argument container. It is assumed
    //! that args[0] constains the alpha parameter, args[1] contains
    //! the kappa parameter, and args[2] contains the beta parameter.
    //!
    explicit Ukf(std::vector<double> args);

    //! @brief Destructor for the Ukf class
    //!
    ~Ukf();

    //! @brief Carries out the correct step in the predict/update cycle.
    //!
    //! @param[in] measurement - The measurement to fuse with our estimate
    //!
    void correct(const Measurement &measurement);

    //! @brief Carries out the predict step in the predict/update cycle.
    //!
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion.
    //!
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //!
    void predict(const double referenceTime, const double delta);

  protected:
    //! @brief Carries out the predict step for the posteriori state of a sigma
    //! point.
    //!
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion.
    //!
    //! @param[in] posterioriState - State of teh sigma point.
    //! @param[in] delta - The time step over which to predict.
    //!
    Eigen::VectorXd predict(Eigen::VectorXd const& posterioriState, double delta);

    //! @brief The UKF sigma points
    //!
    //! Used to sample possible next states during prediction.
    //!
    std::vector<Eigen::VectorXd> sigmaPoints_;

    //! @brief This matrix is used to generate the sigmaPoints_
    //!
    Eigen::MatrixXd weightedCovarSqrt_;

    //! @brief The weights associated with each sigma point when generating
    //! a new state
    //!
    std::vector<double> stateWeights_;

    //! @brief The weights associated with each sigma point when calculating
    //! a predicted estimateErrorCovariance_
    //!
    std::vector<double> covarWeights_;

    //! @brief Used in weight generation for the sigma points
    //!
    double lambda_;

    //! @brief Used to determine if we need to re-compute the sigma
    //! points when carrying out multiple corrections
    //!
    bool uncorrected_;
};

}  // namespace SensorFusion 

#endif  // SENSOR_FUSION_UKF_H
