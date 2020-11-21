#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

sensor_msgs::PointCloud pcl_points;
nav_msgs::OccupancyGrid grid;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr msg) {
    tf::TransformListener tf_;
    laser_geometry::LaserProjection projector_;
    try{
        tf_.waitForTransform("hokuyo", "chassis", (msg->header.stamp+ros::Duration(0.0).fromSec(msg->ranges.size()*msg->time_increment)), ros::Duration(1.0));
        projector_.transformLaserScanToPointCloud("chassis", *msg, pcl_points, tf_);
    }catch (tf::TransformException &ex) {
        return;
    }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_transform");

    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud>("/pcl", 1);
    ros::Subscriber subscriber = nh.subscribe("/scan", 1, &scanCallback);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/custom_map", 1); 

    int width = 200;
    int height = 200;
    double resolution = 0.1;
    ros::Rate sleep_rate(10);

    grid.header.frame_id = "odom";
    grid.info.height = width;
    grid.info.width = height;
    grid.info.resolution = 0.1;
    // grid.info.origin.position.x = static_cast<double>(-width/2);
    // grid.info.origin.position.y = static_cast<double>(-height/2);
    grid.info.origin.position.x = -10;
    grid.info.origin.position.y = -10;
    grid.data.assign((width*height), -1);

    while(ros::ok()) {
        map_publisher.publish(grid);
        publisher.publish(pcl_points);
        ros::spinOnce();
        sleep_rate.sleep();
    }
    return 0;
}