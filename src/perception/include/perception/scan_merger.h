#ifndef PERCEPTION_SCAN_MERGER_H
#define PERCEPTION_SCAN_MERGET_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>


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
       * @brief - 
       * 
       * @param req 
       * @param res 
       * @return true 
       * @return false 
       */
      bool updateParameters(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
      void frontScanCallback(const sensor_msgs::LaserScan::ConstPtr front_scan);
      void rearScanCallback(const sensor_msgs::LaserScan::ConstPtr rear_scan);

      void publishMessage();
      
      /**
       * @brief - node handle (global)
       */
      ros::NodeHandle nh_;
      /**
       * @brief - node handle (private)
       */
      ros::NodeHandle nh_private_;
      /**
       * @brief - Subscribes to front scan data
       */
      ros::Subscriber front_scan_sub_;
      /**
       * @brief - subscribes to rear scan data
       */
      ros::Subscriber rear_scan_sub_;
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
       * @brief - true if front scan messages are received
       */
      bool front_scan_received_;
      /**
       * @brief - true if rear scan messages are received
       */
      bool rear_scan_received_;
      /**
       * @brief - true if there is an error while processing front scan.
       */
      bool front_scan_error_;
      /**
       * @brief - true if there is an error while processing rear scan.
       */
      bool rear_scan_error_;
      /**
       * @brief - stores the converted front laser scan message.
       */
      sensor_msgs::PointCloud2 front_pcl_;
      /**
       * @brief - stores the converted rear laser scan message.
       */
      sensor_msgs::PointCloud2 rear_pcl_;
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