#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "occupancygrid_mapping/ray_tracing.h"
#include <cmath>

namespace sm=sensor_msgs;
namespace rvizT=rviz_visual_tools;
class LaserProject{
    // define namespaces
private:
    ros::Publisher pcl_pub_;
    ros::Subscriber scan_sub_;
    tf::TransformListener sensor_to_map_;
    laser_geometry::LaserProjection projection_;
    std::string fixed_frame_;
    std::string sensor_frame_;
    ros::NodeHandle nh_;
    rvizT::RvizVisualToolsPtr  visual_tools_;
public:
    LaserProject(ros::NodeHandle &n) : fixed_frame_("map"), sensor_frame_("base_scan"), nh_(n)
    {
        visual_tools_.reset(new rvizT::RvizVisualTools("map", "/occupancy_grid_markers"));
        visual_tools_->deleteAllMarkers();
        visual_tools_->enableBatchPublishing();
        pcl_pub_ = nh_.advertise<sm::PointCloud>("pcl", 1);
        scan_sub_ = nh_.subscribe("scan", 1, &LaserProject::callBack, this);
    }
    void callBack(const sm::LaserScan::ConstPtr &scan_in) {
        // delete the previous markers
        visual_tools_->deleteAllMarkers();
        sm::PointCloud pcl_points;
        if (!sensor_to_map_.waitForTransform(fixed_frame_, sensor_frame_,
                                             scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                             ros::Duration(0.1))) {
            ROS_WARN_STREAM("timed out waiting for transform at time " << scan_in->header.stamp);
            return;
        }
        geometry_msgs::PoseStamped sensor_in_map_frame;
        sensor_in_map_frame.header.frame_id = scan_in->header.frame_id;
        sensor_in_map_frame.pose.orientation.w = 1.0;
        sensor_to_map_.transformPose(fixed_frame_, sensor_in_map_frame, sensor_in_map_frame);
        geometry_msgs::Point point = sensor_in_map_frame.pose.position;
//         ROS_INFO_STREAM("Position of laser scan in the world -- x : "<<point.x<<" y : "<<point.y<<" z : "<<point.z);

        projection_.transformLaserScanToPointCloud(fixed_frame_, *scan_in, pcl_points, sensor_to_map_);
        // round the values of sensor pose
        // create a vector pair to get the traced
        vecPair result;
        findRay(ceil(point.x), ceil(point.y), ceil(pcl_points.points[4].x), ceil(pcl_points.points[4].y), result);
        ROS_INFO_STREAM("Size of result : "<<result.size());
        for(auto ele : pcl_points.points) {
            sendMarker(ele.x, ele.y);
        }
        pcl_pub_.publish(pcl_points);
        return;
    }

    void sendMarker(float &x_loc, float &y_loc) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation().x() = x_loc;
        pose.translation().y() = y_loc;
        pose.translation().z() = 3.0;

        this->visual_tools_->publishCuboid(pose, 0.05, 0.05, 0.0001, rvizT::RED);
        this->visual_tools_->trigger();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_transform");
    ros::NodeHandle nh;
    LaserProject laserProject(nh);
    ros::spin();
    return 0;
}