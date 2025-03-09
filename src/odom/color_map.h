// #pragma once
// #include <iostream>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <cocolic/feature_cloud.h>

// #include <utils/my_cloud_type.h>

// class colormap{
// public:
//     ros::Subscriber cur_odom_sub;
//     ros::Subscriber cur_cloud_sub;
//     ros::Subscriber cur_image_sub;

//     ros::Publisher cur_color_map_pub;

// public:
//      colormap(ros::NodeHandle &nh)
//     {
//         cur_color_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/colormap",10);
//         cur_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lio/source_dense_cloud",&cloudcbk(),ros::TransportHints().tcpNoDelay());;
//     }
    

// };