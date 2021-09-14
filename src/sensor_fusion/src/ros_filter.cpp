#include "sensor_fusion/ros_filter.h"
#include "sensor_fusion/filter_utilities.h"
#include "sensor_fusion/ekf.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <limits>


namespace SensorFusion 
{

  template<typename T>
  RosFilter<T>::RosFilter(ros::NodeHandle nh,
                          ros::NodeHandle nh_priv,
                          std::string node_name,
                          std::vector<double> args) :
    disabledAtStartup_(false),
    enabled_(false),
    predictToCurrentTime_(false),
    printDiagnostics_(true),
    publishAcceleration_(false),
    publishTransform_(true),
    resetOnTimeJump_(false),
    smoothLaggedData_(false),
    toggledOn_(true),
    twoDMode_(false),
    useControl_(false),
    dynamicDiagErrorLevel_(diagnostic_msgs::DiagnosticStatus::OK),
    staticDiagErrorLevel_(diagnostic_msgs::DiagnosticStatus::OK),
    frequency_(30.0),
    gravitationalAcc_(9,80665),
    historyLength_(0),
    minFrequency_(frequency_ - 2.0),
    maxFrequency_(frequency_ + 2.0),
    baseLinkFrameId_("base_link"),
    mapFrameId_("map"),
    odomFrameId_("odom"),
    worldFrameId_(odomFrameId_),
    lastDiagTime_(0),
    lastSetPoseTime_(0),
    latestControltime_(0),
    tfTimeoffset_(ros::Duration(0)),
    tfTimeout_(ros::Duration(0)),
    filter_(args),
    nh_(nh),
    nhLocal_(nh_priv),
    diagnosticUpdater_(nh, nh_priv, node_name),
    tfListener_(tfBuffer_)
  {
    stateVariableNames_.push_back("X");
    stateVariableNames_.push_back("Y");
    stateVariableNames_.push_back("Z");
    stateVariableNames_.push_back("ROLL");
    stateVariableNames_.push_back("PITCH");
    stateVariableNames_.push_back("YAW");
    stateVariableNames_.push_back("X_VELOCITY");
    stateVariableNames_.push_back("Y_VELOCITY");
    stateVariableNames_.push_back("Z_VELOCITY");
    stateVariableNames_.push_back("ROLL_VELOCITY");
    stateVariableNames_.push_back("PITCH_VELOCITY");
    stateVariableNames_.push_back("YAW_VELOCITY");
    stateVariableNames_.push_back("X_ACCELERATION");
    stateVariableNames_.push_back("Y_ACCELERATION");
    stateVariableNames_.push_back("Z_ACCELERATION");

    diagnosticUpdater_.setHardwareID("none");
  }

  template<typename T>
  RosFilter<T>::RosFilter(ros::NodeHandle nh, ros::NodeHandle nh_priv, std::vector<double> args) :
      RosFilter<T>::RosFilter(nh, nh_priv, ros::this_node::getName(), args) 
  {}


      

} // namespace SensorFusion