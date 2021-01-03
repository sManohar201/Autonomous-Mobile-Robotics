#ifndef FAKE_LOCALIZER
#define FAKE_LOCALIZER
/**
 * @file fake_localizer.cpp
 * @author Sabari Manohar( sabarimanohar1993@gmail.com)
 * @brief Fake localization node.
 *              Occupancy grid mapping is performed under an assumtion that
 *              the robot location is know. The problem of localization and mapping
 *              goes hand in hand. In this particular implementation, the pose of 
 *              of the robot in the odom frame is transformed to the map frame which
 *              is the base coordinate frame.
 *              This process could be accomplished with the 
 *              static_transform_publisher rostool.
 * @version 0.1
 * @date 2021-01-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace Localization
{
    /**
     * @brief Could be replace to provide Localization. But 
     * currently, it broadcasts an identity transform from "map"
     * "odom"
     */
   class FakeLocalizer {
       public:
        FakeLocalizer(ros::NodeHandle &n, const double time) :
            n_(n), loop_rate_(time)  {
                // broadcast_call_ = n_.createWallTimer(ros::WallDuration(broadcast_time_),
                //         &FakeLocalizer::broadCastTransform, this);
            ROS_INFO("Class initialized");
            this->broadCastTransform();
        }
        /**
         * @brief This will get called at 10 Hz rate. This sends an identity transform
         *      from "map" to "odom" frame.
         * 
         * @param msg 
         */
        void broadCastTransform() {
            ROS_INFO("Transform published");
            while (ros::ok()) {
                geometry_msgs::TransformStamped transform_;
                transform_.header.frame_id = "map";
                transform_.header.stamp = ros::Time::now();
                transform_.child_frame_id = "odom";
                // set identity transform
                transform_.transform.translation.x = 0.0;
                transform_.transform.translation.y = 0.0;
                transform_.transform.translation.z = 0.0;
                transform_.transform.rotation.w = 1.0;
                transform_.transform.rotation.x = 0.0;
                transform_.transform.rotation.y = 0.0;
                transform_.transform.rotation.z = 0.0;

                transcaster_.sendTransform(transform_);
                ros::spinOnce();
                loop_rate_.sleep();
            }
            
        }
       private:
        ros::NodeHandle n_;
        // ros::WallTimer broadcast_call_;
        ros::Rate loop_rate_;
        tf2_ros::TransformBroadcaster transcaster_;
   }; 
} // namespace Localization

#endif