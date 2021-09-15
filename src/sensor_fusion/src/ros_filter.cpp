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

  template<typename T>
  RosFilter<T>::~RosFilter()
  {
    topicSubs_.clear();
  }

  template<typename T>
  void RosFilter<T>::initialize()
  {
    loadParams();
    // TODO: still unfinished
  }

  template<typename T>
  void RosFilter<T>::loadParams()
  {
    /* For diagnostic purposes, collect information about hou many different sources
    * are measuring each absolute pose variables and do not have differential
    * integration enabled.
    * */
    std::map<StateMembers, int> absPoseVarCounts;
    absPoseVarCounts[StateMemberX] = 0;
    absPoseVarCounts[StateMemberY] = 0;
    absPoseVarCounts[StateMemberZ] = 0;
    absPoseVarCounts[StateMemberRoll] = 0;
    absPoseVarCounts[StateMemberPitch] = 0;
    absPoseVarCounts[StateMemberYaw] = 0;

    std::map<StateMember, int> twistVarCounts;
    twistVarCounts[StateMemberVx] = 0;
    twistVarCounts[StateMemberVy] = 0;
    twistVarCounts[StateMemberVz] = 0;
    twistVarCounts[StateMemberVroll] = 0;
    twistVarCounts[StateMemberVpitch] = 0;
    twistVarCounts[StateMemberVyaw] = 0;

    // determine if we'll be printing diagnostic information
    nhLocal_.param("print_diagnostics", printDiagnostics_, true);
    // check for custom gravitational acceleration value
    nhLocal_.param("gravitational_acceleration", gravitationalAcc_, 9.80665);
    // Grab the debug param. If true, the node will produce a lot of output.
    bool debug;
    nhLocal_.param("debug", debug, false);
    if (debug) 
    {
      std::string debugOutFile;

      try 
      {
        nhLocal_.param("debug_out_file", debugOutFile, std::string("sensor_fusion_debug.txt"));
        debugStream_.open(debugOutFile.c_str());

        // make sure we succeeded
        if (debugStream_.is_open())
        {
          filter_.setDebug(debug, &debugStream_);
        }
        else
        {
          ROS_WARN_STREAM("RosFilter::loadParams() - unable to create debug output file " << debugOutFile);
        }
      }
      catch(const std::exception &e)
      {
        ROS_WARN_STREAM("RosFilter::loadParams() - unable to create debug output file " << debugOutFile << ". Error was " 
            e.what() << "\n");
      }
    }
    // these parameters specify the name of the robot's body frame (typically base_link) and odometry frame (typically odom)
    // TODO:  Why defined under local or private nodehandle.
    nhLocal_.param("map_frame", mapFrameId_, std::string("map"));
    nhLocal_.param("odom_frame", odomFrameId_, std::string("odom"));
    nhLocal_.param("base_link_frame", baseLinkFrameId_, std::string("base_link"));
    nhLocal_.param("base_link_frame_output", baseLinkOutputFrameId_, baseLinkFrameId_);

    nhLocal_.param("world_frame", worldFrameId_, odomFrameId_);
    ROS_FATAL_COND(mapFrameId_ == odomFrameId_ ||
                  odomFrameId_ == baseLinkFrameId_ ||
                  mapFrameId_ == baseLinkFrameId_ ||
                  odomFrameId_ == baseLinkOutputFrameId_ ||
                  mapFrameId_ == baseLinkOutputFrameId_, 
                  "Invalid frame  configuration! the values for map_frame, odom_frame, and base_link_frame must be unique. If using a base_link_frame_output values, it must not match the map_frame or odom_frame.");

    std::string tfPrefix = "";
    std::string tfPrefixPath = "";
    if (nhLocal_.searchParam("tf_prefix", tfPrefixPath))
    {
      nhLocal_.getParam(tfPrefixPath, tfPrefix);
    }

    FilterUtilities::appendPrefix(tfPrefix, mapFrameId_);
    FilterUtilities::appendPrefix(tfPrefix, odomFrameId_);
    FilterUtilities::appendPrefix(tfPrefix, baseLinkFrameId_);
    FilterUtilities::appendPrefix(tfPrefix, baseLinkOutputFrameId_);
    FilterUtilities::appendPrefix(tfPrefix, worldFrameId_);

    nhLocal_.param("publish_tf", publishTransform_, true);

    nhLocal_.param("publish_acceleration", publishAcceleration_, false);

    nhLocal_.param("permit_corrected_publication", permitCorrectedPublication_, false);

    // TODO: why future dated?
    double offsetTmp;
    nhLocal_.param("transform_time_offset", offsetTmp, 0.0);

    double timeoutTmp;
    nhLocal_param("transform_timeout", timeoutTmp, 0.0);
    tfTimeout_.fromSec(timeoutTmp);

    double sensorTimeout;
    nhLocal_.param("frequency", frequency_, 30.0);
    nhLocal_.param("sensor_timeout", sensorTimeout, 1.0/frequency_);
    filter_.setSensorTimeout(sensorTimeout);

    nhLocal_.param("two_d_mode", twoDMode_, false);
    // TODO: how exactly done?
    nhLocal_.param("smooth_lagged_data", smoothLaggedData_, false);
    nhLocal_.param("history_length", historyLength_, 0.0);

    nhLocal_.param("reset_on_time_jump", resetOnTimeJump_, false);

    if (!smoothLaggedData_ && ::fabs(historyLength_) > 1e-9)
    {
      ROS_WARN_STREAM("Filter history interval of " << historyLength_ <<
                    " specified, but smooth_lagged_data is set to false. Lagged data will not be smoothed.");
    }
    if (smoothLaggedData_ && historyLength_ < -1e-9)
    {
      ROS_WARN_STREAM("Negative history interval of " << historyLength_ <<
                      " specified. Absolute value will be assumed.");
    }

    historyLength_ = ::fabs(historyLength_);

    nhLocal_.param("predict_to_current_time", predictToCurrentTime_, false);

    // Determine if we're using a control term
    bool stampedControl = false;
    double controlTimeout = sensorTimeout;
    std::vector<int> controlUpdateVector(TWIST_SIZE, 0);
    std::vector<double> accelerationLimits(TWIST_SIZE, 1.0);
    std::vector<double> accelerationGains(TWIST_SIZE, 1.0);
    std::vector<double> decelerationLimits(TWIST_SIZE, 1.0);
    std::vector<double> decelerationGains(TWIST_SIZE, 1.0);

    nhLocal_.param("use_control", useControl_, false);
    nhLocal_.param("stamped_control", stampedControl, false);
    nhLocal_.param("control_timeout", controlTimeout, sensorTimeout);

    if (useControl_)
    {
      if (nhLocal_.getParam("control_config", controlUpdateVector))
      {
        if (controlUpdateVector.size() != TWIST_SIZE)
        {
          ROS_ERROR_STREAM("Control configuration must be of size " << TWIST_SIZE << ". Provided config was of "
            "size " << controlUpdateVector.size() << ". No control term will be used.");
          useControl_ = false;
        }
      }
      else
      {
        ROS_ERROR_STREAM("use_control is set to true, but control_config is missing. No control term will be used.");
        useControl_ = false;
      }

      if (nhLocal_.getParam("acceleration_limits", accelerationLimits))
      {
        if (accelerationLimits.size() != TWIST_SIZE)
        {
          ROS_ERROR_STREAM("Acceleration configuration must be of size " << TWIST_SIZE << ". Provided config was of "
            "size " << accelerationLimits.size() << ". No control term will be used.");
          useControl_ = false;
        }
      }
      else
      {
        ROS_WARN_STREAM("use_control is set to true, but acceleration_limits is missing. Will use default values.");
      }

      if (nhLocal_.getParam("acceleration_gains", accelerationGains))
      {
        const int size = accelerationGains.size();
        if (size != TWIST_SIZE)
        {
          ROS_ERROR_STREAM("Acceleration gain configuration must be of size " << TWIST_SIZE <<
            ". Provided config was of size " << size << ". All gains will be assumed to be 1.");
          std::fill_n(accelerationGains.begin(), std::min(size, TWIST_SIZE), 1.0);
          accelerationGains.resize(TWIST_SIZE, 1.0);
        }
      }

      if (nhLocal_.getParam("deceleration_limits", decelerationLimits))
      {
        if (decelerationLimits.size() != TWIST_SIZE)
        {
          ROS_ERROR_STREAM("Deceleration configuration must be of size " << TWIST_SIZE <<
            ". Provided config was of size " << decelerationLimits.size() << ". No control term will be used.");
          useControl_ = false;
        }
      }
      else
      {
        ROS_INFO_STREAM("use_control is set to true, but no deceleration_limits specified. Will use acceleration "
          "limits.");
        decelerationLimits = accelerationLimits;
      }

      if (nhLocal_.getParam("deceleration_gains", decelerationGains))
      {
        const int size = decelerationGains.size();
        if (size != TWIST_SIZE)
        {
          ROS_ERROR_STREAM("Deceleration gain configuration must be of size " << TWIST_SIZE <<
            ". Provided config was of size " << size << ". All gains will be assumed to be 1.");
          std::fill_n(decelerationGains.begin(), std::min(size, TWIST_SIZE), 1.0);
          decelerationGains.resize(TWIST_SIZE, 1.0);
        }
      }
      else
      {
        ROS_INFO_STREAM("use_control is set to true, but no deceleration_gains specified. Will use acceleration "
          "gains.");
        decelerationGains = accelerationGains;
      }
    }

    bool dynamicProcessNoiseCovariance = false;
    nhLocal_.param("dynamic_process_noise_covariance", dynamicProcessNoiseCovariance, false);
    filter_.setUseDynamicProcessNoiseCovariance(dynamicProcessNoiseCovariance);

    std::vector<double> initialState(STATE_SIZE, 0.0);
    if (nhLocal_.getParam("initial_state", initialState))
    {
      if (initialState.size() != STATE_SIZE)
      {
        ROS_ERROR_STREAM("Initial state must be of size " << STATE_SIZE << ". Provided config was of size " <<
          initialState.size() << ". The initial state will be ignored.");
      }
      else
      {
        Eigen::Map<Eigen::VectorXd> eigenState(initialState.data(), initialState.size());
        filter_.setState(eigenState);
      }
    }

    // Check if the filter should start or not
    nhLocal_.param("disabled_at_startup", disabledAtStartup_, false);
    enabled_ = !disabledAtStartup_;


    // Debugging writes to file
    RF_DEBUG("tf_prefix is " << tfPrefix <<
             "\nmap_frame is " << mapFrameId_ <<
             "\nodom_frame is " << odomFrameId_ <<
             "\nbase_link_frame is " << baseLinkFrameId_ <<
             "\base_link_frame_output is " << baseLinkOutputFrameId_ <<
             "\nworld_frame is " << worldFrameId_ <<
             "\ntransform_time_offset is " << tfTimeOffset_.toSec() <<
             "\ntransform_timeout is " << tfTimeout_.toSec() <<
             "\nfrequency is " << frequency_ <<
             "\nsensor_timeout is " << filter_.getSensorTimeout() <<
             "\ntwo_d_mode is " << std::boolalpha << twoDMode_ <<
             "\nsmooth_lagged_data is " << std::boolalpha << smoothLaggedData_ <<
             "\nhistory_length is " << historyLength_ <<
             "\nuse_control is " << std::boolalpha << useControl_ <<
             "\nstamped_control is " << std::boolalpha << stampedControl <<
             "\ncontrol_config is " << controlUpdateVector <<
             "\ncontrol_timeout is " << controlTimeout <<
             "\nacceleration_limits are " << accelerationLimits <<
             "\nacceleration_gains are " << accelerationGains <<
             "\ndeceleration_limits are " << decelerationLimits <<
             "\ndeceleration_gains are " << decelerationGains <<
             "\ninitial state is " << filter_.getState() <<
             "\ndynamic_process_noise_covariance is " << std::boolalpha << dynamicProcessNoiseCovariance <<
             "\npermit_corrected_publication is " << std::boolalpha << permitCorrectedPublication_ <<
             "\nprint_diagnostics is " << std::boolalpha << printDiagnostics_ << "\n");

    // Create a subscriber for manually setting/resetting pose
    setPoseSub_ = nh_.subscribe("set_pose",
                                1,
                                &RosFilter<T>::setPoseCallback,
                                this, ros::TransportHints().tcpNoDelay(false));

    // Create a service for manually setting/resetting pose
    setPoseSrv_ = nh_.advertiseService("set_pose", &RosFilter<T>::setPoseSrvCallback, this);

    // Create a service for manually enabling the filter
    enableFilterSrv_ = nhLocal_.advertiseService("enable", &RosFilter<T>::enableFilterSrvCallback, this);

    // Create a service for toggling processing new measurements while still publishing
    toggleFilterProcessingSrv_ =
      nhLocal_.advertiseService("toggle", &RosFilter<T>::toggleFilterProcessingCallback, this);

    // Init the last measurement time so we don't get a huge initial delta
    filter_.setLastMeasurementTime(ros::Time::now().toSec());

    // Now pull in each topic to which we want to subscribe.
    // Start with odom.
    size_t topicInd = 0;
    bool moreParams = false;
    do
    {
      // Build the string in the form of "odomX", where X is the odom topic number,
      // then check if we have any parameters with that value. Users need to make
      // sure they don't have gaps in their configs (e.g., odom0 and then odom2)
      std::stringstream ss;
      ss << "odom" << topicInd++;
      std::string odomTopicName = ss.str();
      moreParams = nhLocal_.hasParam(odomTopicName);

      if (moreParams)
      {
        // Determine if we want to integrate this sensor differentially
        bool differential;
        nhLocal_.param(odomTopicName + std::string("_differential"), differential, false);

        // Determine if we want to integrate this sensor relatively
        bool relative;
        nhLocal_.param(odomTopicName + std::string("_relative"), relative, false);

        if (relative && differential)
        {
          ROS_WARN_STREAM("Both " << odomTopicName << "_differential" << " and " << odomTopicName <<
                          "_relative were set to true. Using differential mode.");

          relative = false;
        }

        std::string odomTopic;
        nhLocal_.getParam(odomTopicName, odomTopic);

        // Check for pose rejection threshold
        double poseMahalanobisThresh;
        nhLocal_.param(odomTopicName + std::string("_pose_rejection_threshold"),
                       poseMahalanobisThresh,
                       std::numeric_limits<double>::max());

        // Check for twist rejection threshold
        double twistMahalanobisThresh;
        nhLocal_.param(odomTopicName + std::string("_twist_rejection_threshold"),
                       twistMahalanobisThresh,
                       std::numeric_limits<double>::max());

        // Now pull in its boolean update vector configuration. Create separate vectors for pose
        // and twist data, and then zero out the opposite values in each vector (no pose data in
        // the twist update vector and vice-versa).
        std::vector<int> updateVec = loadUpdateConfig(odomTopicName);
        std::vector<int> poseUpdateVec = updateVec;
        std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
        std::vector<int> twistUpdateVec = updateVec;
        std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

        int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);
        int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);
        int odomQueueSize = 1;
        nhLocal_.param(odomTopicName + "_queue_size", odomQueueSize, 1);

        const CallbackData poseCallbackData(odomTopicName + "_pose", poseUpdateVec, poseUpdateSum, differential,
          relative, poseMahalanobisThresh);
        const CallbackData twistCallbackData(odomTopicName + "_twist", twistUpdateVec, twistUpdateSum, false, false,
          twistMahalanobisThresh);

        bool nodelayOdom = false;
        nhLocal_.param(odomTopicName + "_nodelay", nodelayOdom, false);

        // Store the odometry topic subscribers so they don't go out of scope.
        if (poseUpdateSum + twistUpdateSum > 0)
        {
          topicSubs_.push_back(
            nh_.subscribe<nav_msgs::Odometry>(odomTopic, odomQueueSize,
              boost::bind(&RosFilter::odometryCallback, this, _1, odomTopicName, poseCallbackData, twistCallbackData),
              ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayOdom)));
        }
        else
        {
          std::stringstream stream;
          stream << odomTopic << " is listed as an input topic, but all update variables are false";

          addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                        odomTopic + "_configuration",
                        stream.str(),
                        true);
        }

        if (poseUpdateSum > 0)
        {
          if (differential)
          {
            twistVarCounts[StateMemberVx] += poseUpdateVec[StateMemberX];
            twistVarCounts[StateMemberVy] += poseUpdateVec[StateMemberY];
            twistVarCounts[StateMemberVz] += poseUpdateVec[StateMemberZ];
            twistVarCounts[StateMemberVroll] += poseUpdateVec[StateMemberRoll];
            twistVarCounts[StateMemberVpitch] += poseUpdateVec[StateMemberPitch];
            twistVarCounts[StateMemberVyaw] += poseUpdateVec[StateMemberYaw];
          }
          else
          {
            absPoseVarCounts[StateMemberX] += poseUpdateVec[StateMemberX];
            absPoseVarCounts[StateMemberY] += poseUpdateVec[StateMemberY];
            absPoseVarCounts[StateMemberZ] += poseUpdateVec[StateMemberZ];
            absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
            absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
            absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
          }
        }

        if (twistUpdateSum > 0)
        {
          twistVarCounts[StateMemberVx] += twistUpdateVec[StateMemberVx];
          twistVarCounts[StateMemberVy] += twistUpdateVec[StateMemberVx];
          twistVarCounts[StateMemberVz] += twistUpdateVec[StateMemberVz];
          twistVarCounts[StateMemberVroll] += twistUpdateVec[StateMemberVroll];
          twistVarCounts[StateMemberVpitch] += twistUpdateVec[StateMemberVpitch];
          twistVarCounts[StateMemberVyaw] += twistUpdateVec[StateMemberVyaw];
        }

        RF_DEBUG("Subscribed to " << odomTopic << " (" << odomTopicName << ")\n\t" <<
                 odomTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
                 odomTopicName << "_pose_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
                 odomTopicName << "_twist_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
                 odomTopicName << "_queue_size is " << odomQueueSize << "\n\t" <<
                 odomTopicName << " pose update vector is " << poseUpdateVec << "\t"<<
                 odomTopicName << " twist update vector is " << twistUpdateVec);
      }
    }
    while (moreParams);

    // Repeat for pose
    topicInd = 0;
    moreParams = false;
    do
    {
      std::stringstream ss;
      ss << "pose" << topicInd++;
      std::string poseTopicName = ss.str();
      moreParams = nhLocal_.hasParam(poseTopicName);

      if (moreParams)
      {
        bool differential;
        nhLocal_.param(poseTopicName + std::string("_differential"), differential, false);

        // Determine if we want to integrate this sensor relatively
        bool relative;
        nhLocal_.param(poseTopicName + std::string("_relative"), relative, false);

        if (relative && differential)
        {
          ROS_WARN_STREAM("Both " << poseTopicName << "_differential" << " and " << poseTopicName <<
                          "_relative were set to true. Using differential mode.");

          relative = false;
        }

        std::string poseTopic;
        nhLocal_.getParam(poseTopicName, poseTopic);

        // Check for pose rejection threshold
        double poseMahalanobisThresh;
        nhLocal_.param(poseTopicName + std::string("_rejection_threshold"),
                       poseMahalanobisThresh,
                       std::numeric_limits<double>::max());

        int poseQueueSize = 1;
        nhLocal_.param(poseTopicName + "_queue_size", poseQueueSize, 1);

        bool nodelayPose = false;
        nhLocal_.param(poseTopicName + "_nodelay", nodelayPose, false);

        // Pull in the sensor's config, zero out values that are invalid for the pose type
        std::vector<int> poseUpdateVec = loadUpdateConfig(poseTopicName);
        std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET,
                  poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE,
                  0);
        std::fill(poseUpdateVec.begin() + POSITION_A_OFFSET,
                  poseUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,
                  0);

        int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);

        if (poseUpdateSum > 0)
        {
          const CallbackData callbackData(poseTopicName, poseUpdateVec, poseUpdateSum, differential, relative,
            poseMahalanobisThresh);

          topicSubs_.push_back(
            nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(poseTopic, poseQueueSize,
              boost::bind(&RosFilter::poseCallback, this, _1, callbackData, worldFrameId_, false),
              ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayPose)));

          if (differential)
          {
            twistVarCounts[StateMemberVx] += poseUpdateVec[StateMemberX];
            twistVarCounts[StateMemberVy] += poseUpdateVec[StateMemberY];
            twistVarCounts[StateMemberVz] += poseUpdateVec[StateMemberZ];
            twistVarCounts[StateMemberVroll] += poseUpdateVec[StateMemberRoll];
            twistVarCounts[StateMemberVpitch] += poseUpdateVec[StateMemberPitch];
            twistVarCounts[StateMemberVyaw] += poseUpdateVec[StateMemberYaw];
          }
          else
          {
            absPoseVarCounts[StateMemberX] += poseUpdateVec[StateMemberX];
            absPoseVarCounts[StateMemberY] += poseUpdateVec[StateMemberY];
            absPoseVarCounts[StateMemberZ] += poseUpdateVec[StateMemberZ];
            absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
            absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
            absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
          }
        }
        else
        {
          ROS_WARN_STREAM("Warning: " << poseTopic << " is listed as an input topic, "
                          "but all pose update variables are false");
        }

        RF_DEBUG("Subscribed to " << poseTopic << " (" << poseTopicName << ")\n\t" <<
                 poseTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
                 poseTopicName << "_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
                 poseTopicName << "_queue_size is " << poseQueueSize << "\n\t" <<
                 poseTopicName << " update vector is " << poseUpdateVec);
      }
    }
    while (moreParams);

    // Repeat for twist
    topicInd = 0;
    moreParams = false;
    do
    {
      std::stringstream ss;
      ss << "twist" << topicInd++;
      std::string twistTopicName = ss.str();
      moreParams = nhLocal_.hasParam(twistTopicName);

      if (moreParams)
      {
        std::string twistTopic;
        nhLocal_.getParam(twistTopicName, twistTopic);

        // Check for twist rejection threshold
        double twistMahalanobisThresh;
        nhLocal_.param(twistTopicName + std::string("_rejection_threshold"),
                       twistMahalanobisThresh,
                       std::numeric_limits<double>::max());

        int twistQueueSize = 1;
        nhLocal_.param(twistTopicName + "_queue_size", twistQueueSize, 1);

        bool nodelayTwist = false;
        nhLocal_.param(twistTopicName + "_nodelay", nodelayTwist, false);

        // Pull in the sensor's config, zero out values that are invalid for the twist type
        std::vector<int> twistUpdateVec = loadUpdateConfig(twistTopicName);
        std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

        int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);

        if (twistUpdateSum > 0)
        {
          const CallbackData callbackData(twistTopicName, twistUpdateVec, twistUpdateSum, false, false,
            twistMahalanobisThresh);

          topicSubs_.push_back(
            nh_.subscribe<geometry_msgs::TwistWithCovarianceStamped>(twistTopic, twistQueueSize,
              boost::bind(&RosFilter<T>::twistCallback, this, _1, callbackData, baseLinkFrameId_),
              ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayTwist)));

          twistVarCounts[StateMemberVx] += twistUpdateVec[StateMemberVx];
          twistVarCounts[StateMemberVy] += twistUpdateVec[StateMemberVy];
          twistVarCounts[StateMemberVz] += twistUpdateVec[StateMemberVz];
          twistVarCounts[StateMemberVroll] += twistUpdateVec[StateMemberVroll];
          twistVarCounts[StateMemberVpitch] += twistUpdateVec[StateMemberVpitch];
          twistVarCounts[StateMemberVyaw] += twistUpdateVec[StateMemberVyaw];
        }
        else
        {
          ROS_WARN_STREAM("Warning: " << twistTopic << " is listed as an input topic, "
                          "but all twist update variables are false");
        }

        RF_DEBUG("Subscribed to " << twistTopic << " (" << twistTopicName << ")\n\t" <<
                 twistTopicName << "_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
                 twistTopicName << "_queue_size is " << twistQueueSize << "\n\t" <<
                 twistTopicName << " update vector is " << twistUpdateVec);
      }
    }
    while (moreParams);

    // Repeat for IMU
    topicInd = 0;
    moreParams = false;
    do
    {
      std::stringstream ss;
      ss << "imu" << topicInd++;
      std::string imuTopicName = ss.str();
      moreParams = nhLocal_.hasParam(imuTopicName);

      if (moreParams)
      {
        bool differential;
        nhLocal_.param(imuTopicName + std::string("_differential"), differential, false);

        // Determine if we want to integrate this sensor relatively
        bool relative;
        nhLocal_.param(imuTopicName + std::string("_relative"), relative, false);

        if (relative && differential)
        {
          ROS_WARN_STREAM("Both " << imuTopicName << "_differential" << " and " << imuTopicName <<
                          "_relative were set to true. Using differential mode.");

          relative = false;
        }

        std::string imuTopic;
        nhLocal_.getParam(imuTopicName, imuTopic);

        // Check for pose rejection threshold
        double poseMahalanobisThresh;
        nhLocal_.param(imuTopicName + std::string("_pose_rejection_threshold"),
                       poseMahalanobisThresh,
                       std::numeric_limits<double>::max());

        // Check for angular velocity rejection threshold
        double twistMahalanobisThresh;
        std::string imuTwistRejectionName =
          imuTopicName + std::string("_twist_rejection_threshold");
        nhLocal_.param(imuTwistRejectionName, twistMahalanobisThresh, std::numeric_limits<double>::max());

        // Check for acceleration rejection threshold
        double accelMahalanobisThresh;
        nhLocal_.param(imuTopicName + std::string("_linear_acceleration_rejection_threshold"),
                       accelMahalanobisThresh,
                       std::numeric_limits<double>::max());

        bool removeGravAcc = false;
        nhLocal_.param(imuTopicName + "_remove_gravitational_acceleration", removeGravAcc, false);
        removeGravitationalAcc_[imuTopicName + "_acceleration"] = removeGravAcc;

        // Now pull in its boolean update vector configuration and differential
        // update configuration (as this contains pose information)
        std::vector<int> updateVec = loadUpdateConfig(imuTopicName);

        // sanity checks for update config settings
        std::vector<int> positionUpdateVec(updateVec.begin() + POSITION_OFFSET,
                                           updateVec.begin() + POSITION_OFFSET + POSITION_SIZE);
        int positionUpdateSum = std::accumulate(positionUpdateVec.begin(), positionUpdateVec.end(), 0);
        if (positionUpdateSum > 0)
        {
          ROS_WARN_STREAM("Warning: Some position entries in parameter " << imuTopicName << "_config are listed true, "
                          "but sensor_msgs/Imu contains no information about position");
        }
        std::vector<int> linearVelocityUpdateVec(updateVec.begin() + POSITION_V_OFFSET,
                                                 updateVec.begin() + POSITION_V_OFFSET + LINEAR_VELOCITY_SIZE);
        int linearVelocityUpdateSum = std::accumulate(linearVelocityUpdateVec.begin(),
                                                      linearVelocityUpdateVec.end(),
                                                      0);
        if (linearVelocityUpdateSum > 0)
        {
          ROS_WARN_STREAM("Warning: Some linear velocity entries in parameter " << imuTopicName << "_config are listed "
                          "true, but an sensor_msgs/Imu contains no information about linear velocities");
        }

        std::vector<int> poseUpdateVec = updateVec;
        // IMU message contains no information about position, filter everything except orientation
        std::fill(poseUpdateVec.begin() + POSITION_OFFSET,
                  poseUpdateVec.begin() + POSITION_OFFSET + POSITION_SIZE,
                  0);
        std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET,
                  poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE,
                  0);
        std::fill(poseUpdateVec.begin() + POSITION_A_OFFSET,
                  poseUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,
                  0);

        std::vector<int> twistUpdateVec = updateVec;
        // IMU message contains no information about linear speeds, filter everything except angular velocity
        std::fill(twistUpdateVec.begin() + POSITION_OFFSET,
                  twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE,
                  0);
        std::fill(twistUpdateVec.begin() + POSITION_V_OFFSET,
                  twistUpdateVec.begin() + POSITION_V_OFFSET + LINEAR_VELOCITY_SIZE,
                  0);
        std::fill(twistUpdateVec.begin() + POSITION_A_OFFSET,
                  twistUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,
                  0);

        std::vector<int> accelUpdateVec = updateVec;
        std::fill(accelUpdateVec.begin() + POSITION_OFFSET,
                  accelUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE,
                  0);
        std::fill(accelUpdateVec.begin() + POSITION_V_OFFSET,
                  accelUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE,
                  0);

        int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);
        int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);
        int accelUpdateSum = std::accumulate(accelUpdateVec.begin(), accelUpdateVec.end(), 0);

        // Check if we're using control input for any of the acceleration variables; turn off if so
        if (static_cast<bool>(controlUpdateVector[ControlMemberVx]) && static_cast<bool>(accelUpdateVec[StateMemberAx]))
        {
          ROS_WARN_STREAM("X acceleration is being measured from IMU; X velocity control input is disabled");
          controlUpdateVector[ControlMemberVx] = 0;
        }
        if (static_cast<bool>(controlUpdateVector[ControlMemberVy]) && static_cast<bool>(accelUpdateVec[StateMemberAy]))
        {
          ROS_WARN_STREAM("Y acceleration is being measured from IMU; Y velocity control input is disabled");
          controlUpdateVector[ControlMemberVy] = 0;
        }
        if (static_cast<bool>(controlUpdateVector[ControlMemberVz]) && static_cast<bool>(accelUpdateVec[StateMemberAz]))
        {
          ROS_WARN_STREAM("Z acceleration is being measured from IMU; Z velocity control input is disabled");
          controlUpdateVector[ControlMemberVz] = 0;
        }

        int imuQueueSize = 1;
        nhLocal_.param(imuTopicName + "_queue_size", imuQueueSize, 1);

        bool nodelayImu = false;
        nhLocal_.param(imuTopicName + "_nodelay", nodelayImu, false);

        if (poseUpdateSum + twistUpdateSum + accelUpdateSum > 0)
        {
          const CallbackData poseCallbackData(imuTopicName + "_pose", poseUpdateVec, poseUpdateSum, differential,
            relative, poseMahalanobisThresh);
          const CallbackData twistCallbackData(imuTopicName + "_twist", twistUpdateVec, twistUpdateSum, differential,
            relative, twistMahalanobisThresh);
          const CallbackData accelCallbackData(imuTopicName + "_acceleration", accelUpdateVec, accelUpdateSum,
            differential, relative, accelMahalanobisThresh);

          topicSubs_.push_back(
            nh_.subscribe<sensor_msgs::Imu>(imuTopic, imuQueueSize,
              boost::bind(&RosFilter<T>::imuCallback, this, _1, imuTopicName, poseCallbackData, twistCallbackData,
                accelCallbackData), ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayImu)));
        }
        else
        {
          ROS_WARN_STREAM("Warning: " << imuTopic << " is listed as an input topic, "
                          "but all its update variables are false");
        }

        if (poseUpdateSum > 0)
        {
          if (differential)
          {
            twistVarCounts[StateMemberVroll] += poseUpdateVec[StateMemberRoll];
            twistVarCounts[StateMemberVpitch] += poseUpdateVec[StateMemberPitch];
            twistVarCounts[StateMemberVyaw] += poseUpdateVec[StateMemberYaw];
          }
          else
          {
            absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
            absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
            absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
          }
        }

        if (twistUpdateSum > 0)
        {
          twistVarCounts[StateMemberVroll] += twistUpdateVec[StateMemberVroll];
          twistVarCounts[StateMemberVpitch] += twistUpdateVec[StateMemberVpitch];
          twistVarCounts[StateMemberVyaw] += twistUpdateVec[StateMemberVyaw];
        }

        RF_DEBUG("Subscribed to " << imuTopic << " (" << imuTopicName << ")\n\t" <<
                 imuTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
                 imuTopicName << "_pose_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
                 imuTopicName << "_twist_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
                 imuTopicName << "_linear_acceleration_rejection_threshold is " << accelMahalanobisThresh << "\n\t" <<
                 imuTopicName << "_remove_gravitational_acceleration is " <<
                                 (removeGravAcc ? "true" : "false") << "\n\t" <<
                 imuTopicName << "_queue_size is " << imuQueueSize << "\n\t" <<
                 imuTopicName << " pose update vector is " << poseUpdateVec << "\t"<<
                 imuTopicName << " twist update vector is " << twistUpdateVec << "\t" <<
                 imuTopicName << " acceleration update vector is " << accelUpdateVec);
      }
    }
    while (moreParams);

    // Now that we've checked if IMU linear acceleration is being used, we can determine our final control parameters
    if (useControl_ && std::accumulate(controlUpdateVector.begin(), controlUpdateVector.end(), 0) == 0)
    {
      ROS_ERROR_STREAM("use_control is set to true, but control_config has only false values. No control term "
        "will be used.");
      useControl_ = false;
    }

    // If we're using control, set the parameters and create the necessary subscribers
    if (useControl_)
    {
      latestControl_.resize(TWIST_SIZE);
      latestControl_.setZero();

      filter_.setControlParams(controlUpdateVector, controlTimeout, accelerationLimits, accelerationGains,
        decelerationLimits, decelerationGains);

      if (stampedControl)
      {
        controlSub_ = nh_.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 1, &RosFilter<T>::controlCallback, this);
      }
      else
      {
        controlSub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &RosFilter<T>::controlCallback, this);
      }
    }

    /* Warn users about:
    *    1. Multiple non-differential input sources
    *    2. No absolute *or* velocity measurements for pose variables
    */
    if (printDiagnostics_)
    {
      for (int stateVar = StateMemberX; stateVar <= StateMemberYaw; ++stateVar)
      {
        if (absPoseVarCounts[static_cast<StateMembers>(stateVar)] > 1)
        {
          std::stringstream stream;
          stream <<  absPoseVarCounts[static_cast<StateMembers>(stateVar - POSITION_OFFSET)] <<
              " absolute pose inputs detected for " << stateVariableNames_[stateVar] <<
              ". This may result in oscillations. Please ensure that your variances for each "
              "measured variable are set appropriately.";

          addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                        stateVariableNames_[stateVar] + "_configuration",
                        stream.str(),
                        true);
        }
        else if (absPoseVarCounts[static_cast<StateMembers>(stateVar)] == 0)
        {
          if ((static_cast<StateMembers>(stateVar) == StateMemberX &&
               twistVarCounts[static_cast<StateMembers>(StateMemberVx)] == 0) ||
              (static_cast<StateMembers>(stateVar) == StateMemberY &&
               twistVarCounts[static_cast<StateMembers>(StateMemberVy)] == 0) ||
              (static_cast<StateMembers>(stateVar) == StateMemberZ &&
               twistVarCounts[static_cast<StateMembers>(StateMemberVz)] == 0 &&
               twoDMode_ == false) ||
              (static_cast<StateMembers>(stateVar) == StateMemberRoll &&
               twistVarCounts[static_cast<StateMembers>(StateMemberVroll)] == 0 &&
               twoDMode_ == false) ||
              (static_cast<StateMembers>(stateVar) == StateMemberPitch &&
               twistVarCounts[static_cast<StateMembers>(StateMemberVpitch)] == 0 &&
               twoDMode_ == false) ||
              (static_cast<StateMembers>(stateVar) == StateMemberYaw &&
               twistVarCounts[static_cast<StateMembers>(StateMemberVyaw)] == 0))
          {
            std::stringstream stream;
            stream << "Neither " << stateVariableNames_[stateVar] << " nor its "
                      "velocity is being measured. This will result in unbounded "
                      "error growth and erratic filter behavior.";

            addDiagnostic(diagnostic_msgs::DiagnosticStatus::ERROR,
                          stateVariableNames_[stateVar] + "_configuration",
                          stream.str(),
                          true);
          }
        }
      }
    }

    // Load up the process noise covariance (from the launch file/parameter server)
    Eigen::MatrixXd processNoiseCovariance(STATE_SIZE, STATE_SIZE);
    processNoiseCovariance.setZero();
    XmlRpc::XmlRpcValue processNoiseCovarConfig;

    if (nhLocal_.hasParam("process_noise_covariance"))
    {
      try
      {
        nhLocal_.getParam("process_noise_covariance", processNoiseCovarConfig);

        ROS_ASSERT(processNoiseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

        int matSize = processNoiseCovariance.rows();

        for (int i = 0; i < matSize; i++)
        {
          for (int j = 0; j < matSize; j++)
          {
            try
            {
              // These matrices can cause problems if all the types
              // aren't specified with decimal points. Handle that
              // using string streams.
              std::ostringstream ostr;
              ostr << processNoiseCovarConfig[matSize * i + j];
              std::istringstream istr(ostr.str());
              istr >> processNoiseCovariance(i, j);
            }
            catch(XmlRpc::XmlRpcException &e)
            {
              throw e;
            }
            catch(...)
            {
              throw;
            }
          }
        }

        RF_DEBUG("Process noise covariance is:\n" << processNoiseCovariance << "\n");
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading sensor config: " <<
                         e.getMessage() <<
                         " for process_noise_covariance (type: " <<
                         processNoiseCovarConfig.getType() << ")");
      }

      filter_.setProcessNoiseCovariance(processNoiseCovariance);
    }

    // Load up the process noise covariance (from the launch file/parameter server)
    Eigen::MatrixXd initialEstimateErrorCovariance(STATE_SIZE, STATE_SIZE);
    initialEstimateErrorCovariance.setZero();
    XmlRpc::XmlRpcValue estimateErrorCovarConfig;

    if (nhLocal_.hasParam("initial_estimate_covariance"))
    {
      try
      {
        nhLocal_.getParam("initial_estimate_covariance", estimateErrorCovarConfig);

        ROS_ASSERT(estimateErrorCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

        int matSize = initialEstimateErrorCovariance.rows();

        for (int i = 0; i < matSize; i++)
        {
          for (int j = 0; j < matSize; j++)
          {
            try
            {
              // These matrices can cause problems if all the types
              // aren't specified with decimal points. Handle that
              // using string streams.
              std::ostringstream ostr;
              ostr << estimateErrorCovarConfig[matSize * i + j];
              std::istringstream istr(ostr.str());
              istr >> initialEstimateErrorCovariance(i, j);
            }
            catch(XmlRpc::XmlRpcException &e)
            {
              throw e;
            }
            catch(...)
            {
              throw;
            }
          }
        }

        RF_DEBUG("Initial estimate error covariance is:\n" << initialEstimateErrorCovariance << "\n");
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading initial_estimate_covariance (type: " <<
                         estimateErrorCovarConfig.getType() <<
                         "): " <<
                         e.getMessage());
      }
      catch(...)
      {
        ROS_ERROR_STREAM(
          "ERROR reading initial_estimate_covariance (type: " << estimateErrorCovarConfig.getType() << ")");
      }

      filter_.setEstimateErrorCovariance(initialEstimateErrorCovariance);
    }
  }

} 