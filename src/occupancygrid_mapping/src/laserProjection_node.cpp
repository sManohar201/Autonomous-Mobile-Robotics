#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

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
        sm::PointCloud pcl_points;
        if (!sensor_to_map_.waitForTransform(fixed_frame_, sensor_frame_,
                                             scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                             ros::Duration(0.1))) {
            ROS_WARN_STREAM("timed out waiting for transform at time " << scan_in->header.stamp);
            return;
        }
        projection_.transformLaserScanToPointCloud(fixed_frame_, *scan_in, pcl_points, sensor_to_map_);
//        ROS_INFO_STREAM("Info on Laser --> Laser scan size : " << scan_in->ranges.size());
//        ROS_INFO_STREAM("Info on pcl -> Pcl point size : " << pcl_points.points[0] << " Pcl channel size : " << pcl_points.channels.size());
        pcl_pub_.publish(pcl_points);
        sendMarker(pcl_points.points[4].x, pcl_points.points[4].y);
        return;
    }

    void sendMarker(float &x_loc, float &y_loc) {
        visual_tools_->deleteAllMarkers();
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