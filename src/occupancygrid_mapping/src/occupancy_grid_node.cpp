/**
 * \file
 * 
 * Ros node to construct occupancy grid.
 * 
 * \author Sabari Manohar
 */
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <boost/circular_buffer.hpp>
// temporarily add geometry, navigation and sensor messages to the header file
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancygrid_mapping/LocalizedCloud.h>

// TODO:: change all occurence of tf to tf2

namespace occupancy_map {
    namespace gm=geometry_msgs;
    namespace sm=sensor_msgs;
    namespace gu=occupancygrid_mapping;
    namespace nm=nav_msgs;

    typedef boost::mutex::scoped_lock Lock;
    typedef boost::shared_ptr<gu::LocalizedCloud> CloudPtr;
    typedef boost::shared_ptr<gu::LocalizedCloud const> CloudConstPtr;
    typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;

    using std::string;
    using std::vector;

    class OccupancyGridNode {
        public:
           OccupancyGridNode();

        private:
            void scanCallback(const sm::LaserScan::ConstPtr &scan);
            void buildGrid(const ros::WallTimerEvent &event);

            ros::NodeHandle nh_;
            const unsigned history_length_;
            const double resolution_;
            const string map_frame_;
            const string sensor_frame_;
            const double grid_construction_interval_;

            tf::TransformListener tf_;
            ros::Subscriber scan_sub_;
            ros::Publisher grid_pub_;
            ros::WallTimer build_grid_timer_;
            boost::mutex mutex_;

            CloudBuffer clouds_;
            CloudConstPtr last_cloud_;
    };

    // define a function to return private parameters
    template<class T>
    T getPrivateParam(const string& name, const T& default_value) {
        ros::NodeHandle nh("~");
        T value;
        nh.param(name, value, default_value);
        return value;
    }

    OccupancyGridNode::OccupancyGridNode () :
        history_length_(getPrivateParam("history_length", 100)), resolution_(getPrivateParam("resolution", 0.1)),
        map_frame_("map"), sensor_frame_("base_scan"), 
        grid_construction_interval_(getPrivateParam("grid_construction_interval", 0.3)),
        scan_sub_(nh_.subscribe("scan", 1, &OccupancyGridNode::scanCallback, this)),
        grid_pub_(nh_.advertise<nm::OccupancyGrid>("final_gird", 1)),
        build_grid_timer_(nh_.createWallTimer(ros::WallDuration(grid_construction_interval_), 
                    &OccupancyGridNode::buildGrid, this)), clouds_(history_length_) {}
    
    void OccupancyGridNode::scanCallback(const sm::LaserScan::ConstPtr &msg) {
        try {
            // first step is to transform between sensor frame and map frame (map_frame_)
            if (!tf_.waitForTransform(map_frame_, sensor_frame_, 
                    msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0))) {
                ROS_WARN_STREAM ("Timed out waiting for transform form " << sensor_frame_ << " to " 
                             << map_frame_ << " at " << msg->header.stamp.toSec());
                return;
            }
            // get the current sensor position
            gm::PoseStamped sensor_world_pose;
            sensor_world_pose.pose.orientation.w = 1.0;
            sensor_world_pose.header.frame_id = msg->header.frame_id;

            tf_.transformPose(map_frame_, sensor_world_pose, sensor_world_pose);

            // project scan message  from sensor frame to map frame
            sm::PointCloud map_frame_cloud;
            laser_geometry::LaserProjection projector_;
            projector_.transformLaserScanToPointCloud(map_frame_, *msg, map_frame_cloud, tf_);

            // another cloud point in the sensor frame
            sm::PointCloud sensor_frame_cloud;
            tf_.transformPointCloud(sensor_frame_, msg->header.stamp, map_frame_cloud, map_frame_, sensor_frame_cloud);

            // construct a localized cloud
            CloudPtr loc_cloud(new gu::LocalizedCloud());
            loc_cloud->cloud.points = sensor_frame_cloud.points;
            loc_cloud->sensor_pose = sensor_world_pose.pose;
            loc_cloud->header.frame_id = map_frame_;
            Lock lock(mutex_);
            last_cloud_ = loc_cloud;
        } catch (tf::TransformException &e) {
            ROS_INFO ("Not saving scan due to tf lookup exception: %s", e.what());
        }
    }

    void OccupancyGridNode::buildGrid(const ros::WallTimerEvent &event) {
        if (last_cloud_) {
            {
                Lock lock(mutex_);
                clouds_.push_back(last_cloud_);

                last_cloud_.reset();
            }
            ROS_DEBUG_NAMED("build_grid", "Building grid with the latest cloud points of total size: %zu", clouds_.size());
            nm::MapMetaData info;
            info.origin.position.x = -4;
            info.origin.position.y = -4;
            info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
            info.resolution = resolution_;
            info.width = 80;
            info.height = 80;
            nm::OccupancyGrid fake_grid;
            fake_grid.header.stamp = ros::Time::now();
            fake_grid.header.frame_id = map_frame_;
            vector<signed char> dummy(80*80, -1);
            dummy[0] = 100;
            
            fake_grid.data = dummy; 
            fake_grid.info = info;
            ROS_DEBUG_NAMED ("build_grid", "Done building grid");
            grid_pub_.publish(fake_grid);
        }
    }
} // namespace occupancy_map

int main(int argc, char **argv) {
    ros::init(argc, argv, "occupancy_gird_node");
    ROS_INFO("Start NOde");
    occupancy_map::OccupancyGridNode node;
    ros::spin();
    return 0; 
}
