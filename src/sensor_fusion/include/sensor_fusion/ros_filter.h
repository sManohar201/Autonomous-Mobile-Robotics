#ifndef SENSOR_FUSION_ROS_FILTER_H
#define SENSOR_FUSION_ROS_FILTER_H

#include "sensor_fusion/filter_common.h"
#include "sensor_fusion/filter_base.h"
#include "sensor_fusion/ros_filter_utilities.h"

#include <sensor_fusion/SetPose.h>
#include <sensor_fusion/ToggleFilterProcessing.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <XmlRpcException.h>

#include <Eigen/Dense>

#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <string>
#include <vector>
#include <deque>


namespace SensorFusion 
{

  struct CallbackData 
  {
    CallbackData(const std::string &topicName,
                 const std::vector<int> &updateVector,
                 const int updateSum,
                 const bool differential,
                 const bool relative,
                 const bool rejectionThreshold) : 
      topicName_(topicName), 
      updateVector_(updateVector), 
      updateSum_(updateSum),
      differential_(differential),
      relative_(relative),
      rejectionThreshold_(rejectionThreshold)
    {}

    std::string topicName_;
    std::vector<int> updateVector_;
    int updateSum_;
    bool differential_;
    bool relative_;
    double rejectionThreshold_;
    
  };
  
  typedef std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>, Measurement> MeasurementQueue;
  typedef std::deque<MeasurementPtr> MeasurementHistoryDeque;
  typedef std::deque<FilterStatePtr> FilterStateHistoryDeque;


  template<class T> class RosFilter 
  {
    public:
      /**
       * @brief Construct a new Ros Filter object
       * 
       * @param[in] nh - node handle to create publisher and subscriber.
       * @param[in] nh_priv - node handle which is private to deal with parameters.
       * @param[in] node_name - name to initialize the node.
       * @param args 
       */
      explicit RosFilter(ros::NodeHandle nh,
                         ros::NodeHandle nh_priv,
                         std::string node_name,
                         std::vector<double> args = std::vector<double>());
                        
      explicit RosFilter(ros::NodeHandle nh, ros::NodeHandle nh_priv, std::vector<double> args = std::vector<double>());

      ~RosFilter();

      void initialize();

      void loadParams();

      void setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
      /**
       * @brief Start the filter disabled at startup
       *  if this is true, the filter reads parameters and prepares publishers and subscribers
       * but does not integrate new messages into the state vector. 
       * The filter can be enables later using a service. 
       */
      bool disabledAtStartup_;
      /**
       * @brief  Whether the filter is enabled or not. See disabledAtStartup_.
       */
      bool enabled_;
      /**
       * @brief whether we'll allow old measurements to cause a re-publication of the updated state.
       */
      bool permitCorrectedPublication_;
      /**
       * @brief By default, the filter predicts and corrects up to the time of the latest measurement.
       * If this is set to true, the filter does the same, but then also predicts up to the crrent
       * time step. 
       */
      bool predictToCurrentTime_;
      /**
       * @brief Whether or not we print diagnostic messages to the /diagnostics topic
       */
      bool printDiagnostics_;
      /**
       * @brief whether we publish the acceleration.
       */
      bool publishAcceleration_;
      /**
       * @brief Whether we publish the transform from the world_frame to the base_link_frame.
       */
      bool publishTransform_;
      /**
       * @brief Whether to reset the filters when backwards jump in time is detected.
       * This is usually the case when logs are being used and a jump in the log is 
       * done or if a log file restarts from the beginning.
       */
      bool resetOnTimeJump_;
      /**
       * @brief whether or not we use smoothing.
       */
      bool smoothLaggedData_;
      /**
       * @brief whether the filter should process new measurements or not.
       */
      bool toggledOn_;
      /**
       * @brief Whether or not we're in 2D mode.
       * if this is true, the filter binds all 3d variables (z, roll, pitch, and 
       * their respective velocities) to 0 for every measurement.
       */
      bool twoDMode_;
      /**
       * @brief Whether or not we use a control term
       */
      bool useControl_;
      /**
       * @brief The max dynamic diagnostic level.
       */
       //  TODO: What is this? 
      int dynamicDiagErrorLevel_;
      /**
       * @brief The max static diagnostic level.
       */
      // TODO: again what is this?
      int staticDiagErrorLevel_;
      /**
       * @brief the frequency of the run loop
       */
      double frequency_;
      /**
       * @brief What is the acceleration in Z due to gravity (m/s^2)? default is +9.80665.
       */
      double gravitationalAcc_;
      /**
       * @brief The depth of the history we track for smoothing/delayed measurement processing
       * this is the guaranteed minimum buffer size for which previous states and 
       * measurements are kept. 
       */
      double historyLength_;
      /**
       * @brief minimal frequency
       */
      double minFrequency_;
      /**
       * @brief maximal frequency
       */
      double maxFrequency_;
      /**
       * @brief tf frame name for the robot's body frame
       */
      std::string baseLinkFrameId_;
      /**
       * @brief tf frame name for the robot's body frame
       * When the final state is computed, we "override"  the output transform and message to 
       * have this frame for its child_frame_id. This helps to enable disconnected TF trees when 
       * multiple EKF instances are being run.
       */
      // TODO: understand this better!
      std::string baseLinkOutputFrameId_;
      /**
       * @brief tf frame name for the robot's map frame
       */
      std::string mapFrameId_;
      /**
       * @brief tf frame name for the robot's odometry frame
       */
      std::string odomFrameId_;
      /**
       * @brief tf frame name that is the parent frame of the 
       * transform that this node will calculate and broadcast.
       */
      std::string worldFrameId_;
      /**
       * @brief used for outputting debug messages
       */
      std::ofstream debugStream_;
      /**
       * @brief Contains the state vector variables names in string format
       */
      std::vector<std::string> stateVariableNames_;
      /**
       * @brief Vector to hold our subscribers until they go out of scope.
       */
      std::vector<ros::Subscriber> topicSubs_;
      /**
       * @brief this object accumulates dynamic diagnostics, e.g., diagnostics relating
       * to sensor data.
       * the values are considered transient and are cleared at every iteration.
       */
      std::map<std::string, std::string> dynamicDiagnostics_;
      /**
       * @brief Stores the first measurement from each topic for relative measurements
       * When a give sensor is being integrated in relative mode, its first measurement
       * is effectively treated as an offset, and future measurements have this first measurement
       * removed before they are fused. This variable stores the initial measurements. 
       * Note that this is different from using differential mode, as in differential mode, 
       * pose data is converted to twist data, resulting in boundless error growth for the 
       * variables being fused. with relative measurements, the vehicle will start with a 0
       * heading and position, but the measurements are still fused absolutely. 
       */
      // TODO: find more details about differential mode.?
      std::map<std::string, tf2::Transform> initialMeasurements_;
      /**
       * @brief Store the last time a message from each topic was received
       * if we're getting messages rapidly, we may accidentally get an older
       * message arriving after a newer one. This variable keeps track of the most 
       * resent message time for each subscribed message topic. We also use it 
       * when listening to odometry messages to determine if we should be using 
       * messages from that topic. 
       */
      std::map<std::string, ros::Time> lastMessageTimes_;
      /**
       * @brief We also need the previous covariance matrix for differential data
       */
      std::map<std::string, Eigen::MatrixXd> previousMeasurementCovariances_;
      /**
       * @brief last call of periodicUpdate
       */
      ros::Time lastDiagTime_;
      ros::Time lastSetPoseTime_;
      ros::Time latestControltime_;
      ros::Duration tfTimeoffset_;
      ros::Duration tfTimeout_;

      T filter_;

      ros::NodeHandle nh_;
      ros::NodeHandle nhLocal_;

      diagnostic_updater::Updater diagnosticUpdater_;

      tf2_ros::TransformListener tfListener_;
      tf2_ros::Buffer tfBuffer_;

  };
} // namespace SensorFusion

#endif