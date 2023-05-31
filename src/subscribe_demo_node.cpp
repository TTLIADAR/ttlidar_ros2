/*
 *  TTldiar System
 *  Driver Interface
 *
 *  Copyright 2023TT Team
 *  All rights reserved. 
 *
 *	Author: Titans  2023-2-21
 *
 */


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)

static void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int count = scan->scan_time / scan->time_increment;
    printf("recv a laser scan %s[%d]*********************\n", scan->header.frame_id.c_str(), count);
    printf("angle_range, %f, %f     ***************\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        printf(": [%f, %f]\n", degree, scan->ranges[i]);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); 

    auto node = rclcpp::Node::make_shared("subscribe_demo_node");

    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                            "scan", rclcpp::SensorDataQoS(), scanCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}


