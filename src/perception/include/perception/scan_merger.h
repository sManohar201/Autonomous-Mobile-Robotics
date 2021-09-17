#ifndef PERCEPTION_SCAN_MERGER_H
#define PERCEPTION_SCAN_MERGET_H


#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>

#include <laser_geometry/laser_geometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "perception/perception_utilities.h"

namespace perception 
{

  class ScanMerger
  {

    public:
      /**
       * @brief Construct a new Scan Merger object
       * 
       * @param[in] nh - ros global node handle.
       * @param[in] nh_local - private node handle.
       */
      ScanMerger(ros::NodeHandle &nh, ros::NodeHandle &nh_local);
      /**
       * @brief Destroy the Scan Merger object
       */
      ~ScanMerger();
    private:
      /**
       * @brief - updates the parameters which controls the activation
       * of the node. 
       * 
       * @param[in] req - request type for service.
       * @param[out] res - response type for service.
       * @return - true if the service was completed successfully. 
       */
      bool updateParameters(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
      /**
       * @brief - mannually initialize the parameters and start the publishers
       */
      void initialize() { std_srvs::Empty empt; updateParameters(empt.request, empt.response); }
      /**
       * @brief - Synchronized callback for front and rear scan. 
       * 
       * @param[in] front_scan - input laser scan from the front lidar.
       * @param[in] rear_scan - input laser scan from the rear lidar.
       */
      void subscriberCallback(const sensor_msgs::LaserScan::ConstPtr &front_scan, 
                              const sensor_msgs::LaserScan::ConstPtr &rear_scan);
      /**
       * @brief - node handle (global)
       */
      ros::NodeHandle nh_;
      /**
       * @brief - node handle (private)
       */
      ros::NodeHandle nh_private_;
      /**
       * @brief - server to setup all the parameters.
       */
      ros::ServiceServer params_srv_;
      /**
       * @brief - subscriber wrapper from message filter - front scan
       */
      message_filters::Subscriber<sensor_msgs::LaserScan> front_scan_;
      /**
       * @brief - subscriber wrapper from message filter - rear scan
       */
      message_filters::Subscriber<sensor_msgs::LaserScan> rear_scan_;
      /**
       * @brief - policy type for synchronizer - approximate policy
       */
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> laser_sync_policy;
      message_filters::Synchronizer<laser_sync_policy> sync_;
      /**
       * @brief - publishes merged laser scan.
       */
      ros::Publisher laser_scan_pub_;
      /**
       * @brief - publishes merged pcl point cloud.
       */
      ros::Publisher pcl_scan_pub_; 
      /**
       * @brief - laser geometry object to convert lacer scan message 
       * to point cloud
       */
      laser_geometry::LaserProjection projector_;
      /**
       * @brief - tf2 api's to perform transformations.
       */
      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;
      /**
       * @brief - stores the converted front laser scan message.
       */
      sensor_msgs::PointCloud2 front_pcl_;
      /**
       * @brief - stores the converted rear laser scan message.
       */
      sensor_msgs::PointCloud2 rear_pcl_;
      /**
       * @brief - target frame for the laser transformation.
       */
      std::string target_frame_;
      /**
       * @brief - check if the node is active
       */
      bool p_active_;
      /**
       * @brief - true if everything is fine to publish laser scan msgs. 
       */
      bool p_publish_scan_;
      /**
       * @brief - true if everything is fine to publish point cloud message
       */
      bool p_publish_pcl_;
  };

} // namespace perception

#endif