#include "sensor_fusion/filter_base.h"
#include "sensor_fusion/filter_common.h"

#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>


namespace SensorFusion {

  FilterBase::FilterBase(): 
    initialized_(false),
    useControl_(false),
    useDynamicProcessNoiseCovariance_(false),
    lastMeasurementTime_(0.0),
    latestControlTime_(0.0),
    controlTimeout_(0.0),
    sensorTimeout_(0.0),
    controlUpdateVector_(TWIST_SIZE, 0),
    accelerationGains_(TWIST_SIZE, 0.0),
    accelerationLimits_(TWIST_SIZE, 0.0),
    decelerationLimits_(TWIST_SIZE, 0.0),
    controlAcceleration_(TWIST_SIZE),
    lastestControl_(TWIST_SIZE),
    predictState_(STATE_SIZE),
    state_(STATE_SIZE),
    covarianceEpsilon_(STATE_SIZE, STATE_SIZE),
    dynamicProcessNoiseCovariance_(STATE_SIZE, STATE_SIZE),
    estimateErrorCovariance_(STATE_SIZE, STATE_SIZE),
    identity_(STATE_SIZE, STATE_SIZE),
    processNoiseCovariance_(STATE_SIZE, STATE_SIZE),
    transferFunction_(STATE_SIZE, STATE_SIZE),
    transferFunctionJacobian_(STATE_SIZE, STATE_SIZE),
    debugStream_(nullptr),
    debug_(false)
  {}
}