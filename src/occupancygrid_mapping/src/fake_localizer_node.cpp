/**
 * @file fake_localizer_node.cpp
 * @author Sabari Manohar (sabarimanohar1993@gmail.com)
 * @brief Fake localizer node, instantiates and calls fake_localizer class. 
 * @version 0.1
 * @date 2021-01-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include "occupancygrid_mapping/fake_localizer.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_localizer");
    ros::NodeHandle n;
    ROS_INFO("Transform node called");
    Localization::FakeLocalizer(n, 10);
    // ros::spin();
    return 0;
}