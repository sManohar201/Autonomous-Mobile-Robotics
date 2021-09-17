#include "perception/scan_merger.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_merger", ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Scans merger]: Initializing node");
    perception::ScanMerger sm(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Scans Merger]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Scans Merger]: Unexpected error");
  }

  return 0;
  
}