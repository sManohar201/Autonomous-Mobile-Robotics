#include "perception/scan_merger.h"
#include <pcl_ros/point_cloud.h>


namespace perception 
{

  ScanMerger::ScanMerger(ros::NodeHandle &nh, ros::NodeHandle &nh_local) :
                        nh_(nh), nh_private_(nh_local),
                        front_scan_(nh_, "/front_laser/scan", 1),
                        rear_scan_(nh_, "/rear_laser/scan", 1),
                        tf_listener_(tf_buffer_),
                        sync_(laser_sync_policy(5), front_scan_, rear_scan_),
                        target_frame_("chassis_link")
  {
    p_active_ = false;
    
    params_srv_ = nh_private_.advertiseService("params", &ScanMerger::updateParameters, this);

    initialize();
  }

  ScanMerger::~ScanMerger()
  {
    nh_private_.deleteParam("active");
    nh_private_.deleteParam("publish_scan");
    nh_private_.deleteParam("publish_pcl");
  }

  bool ScanMerger::updateParameters(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    // store the old value of p_active to prev_active
    bool prev_active = p_active_;

    nh_private_.param<bool>("active", p_active_, true);
    nh_private_.param<bool>("publish_scan", p_publish_scan_, false);
    nh_private_.param<bool>("publish_pcl", p_publish_pcl_, true);

    if (p_active_ != prev_active)
    {
      if(p_active_)
      {
        sync_.registerCallback(&ScanMerger::subscriberCallback, this);
        // laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
        pcl_scan_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("automaton/pcl", 10);
      }
      else 
      {
        laser_scan_pub_.shutdown();
        pcl_scan_pub_.shutdown();
      }
    }
    return true;
  }

  void ScanMerger::subscriberCallback(const sensor_msgs::LaserScan::ConstPtr &front_scan, 
                                      const sensor_msgs::LaserScan::ConstPtr &rear_scan)
  {

    ROS_INFO_STREAM("Synchronization difference : " << (front_scan->header.stamp - rear_scan->header.stamp)); 
    
    try
    {
      projector_.transformLaserScanToPointCloud(target_frame_, *front_scan, front_pcl_, tf_buffer_);
    }
    catch(const char* s)
    {
      ROS_FATAL_STREAM("[Front scan convertion]: " << s);
    }

    try
    {
      projector_.transformLaserScanToPointCloud(target_frame_, *rear_scan, rear_pcl_, tf_buffer_);
    }
    catch(const char *s)
    {
      ROS_FATAL_STREAM("[Rear scan convertion]: " << s);
    }

    sensor_msgs::PointCloud2 final_point_cloud;
    pcl::concatenatePointCloud(front_pcl_, rear_pcl_, final_point_cloud);
    pcl_scan_pub_.publish(final_point_cloud);

    ROS_INFO_STREAM("PCL creation completed.");
  }

}