#include "perception/scan_merger.h"

namespace perception 
{

  ScanMerger::ScanMerger(ros::NodeHandle &nh, ros::NodeHandle &nh_local) :
                        nh_(nh), nh_private_(nh_local)
  {
    p_active_ = false;

    front_scan_received_ = false;
    rear_scan_received_ = false;

    front_scan_error_ = false;
    rear_scan_error_ = false;

    params_srv_ = nh_private_.advertiseService("params", &ScanMerger::updateParameters, this);

    initialize();
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
        front_scan_sub_ = nh_.subscribe("front_scan", 10, &ScanMerger::frontScanCallback, this);
        rear_scan_sub_ = nh_.subscribe("rear_scan", 10, &ScanMerger::rearScanCallback, this);
        laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
        pcl_scan_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("pcl", 10);
      }
      else 
      {
        front_scan_sub_.shutdown();
        rear_scan_sub_.shutdown();
        laser_scan_pub_.shutdown();
        pcl_scan_pub_.shutdown();
      }
    }
    return true;
  }


}