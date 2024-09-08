#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include "net.h"
#include "mat.h"
#include "opencv2/opencv.hpp"
#include "chrono"
#include "tracking.h"

#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/png_io.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <livox_ros_driver2/CustomMsg.h>

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <omp.h>

// #define IMG_H 240
// #define IMG_W 320

using namespace std;
class ncnn_image
{
    public:
    
    ncnn_image() { 
        
         net.opt.num_threads=12;
         net.load_param("/home/hong/slam/lvio/src/LICO-mid360-main/LICO-mid360-main/models/model.param");
         net.load_model("/home/hong/slam/lvio/src/LICO-mid360-main/LICO-mid360-main/models/model.bin");    

    }

    ncnn::Net net;
    corner_tracking tracker;
     

    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1.0/255.0, 1.0/255.0, 1.0/255.0};
    const float mean_vals_inv[3] = {0, 0, 0};
    const float norm_vals_inv[3] = {255.f, 255.f, 255.f};

   

  std::vector<cv::Point2f> ncnn_solve(cv::Mat mat_in)
  {
    int IMG_H = mat_in.rows;
    int IMG_W = mat_in.cols;

    cv::Mat score(IMG_H, IMG_W, CV_8UC1);
    cv::Mat desc(IMG_H, IMG_W, CV_8UC3);
    ncnn::Mat in;
    ncnn::Mat out1, out2;

    
    ncnn::Extractor ex = net.create_extractor();
    ex.set_light_mode(true);
    // ex.set_num_threads(4);

    // cv::resize(mat_in, mat_in, cv::Size(IMG_W, IMG_H));

    //////////////////////////  opencv image to ncnn mat  //////////////////////////
    // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    in = ncnn::Mat::from_pixels(mat_in.data, ncnn::Mat::PIXEL_BGR, mat_in.cols, mat_in.rows);
    in.substract_mean_normalize(mean_vals, norm_vals);

    //////////////////////////  ncnn forward  //////////////////////////

    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

    ex.input("input", in);
    ex.extract("score", out1);
    ex.extract("descriptor", out2);

    //////////////////////////  ncnn mat to opencv image  //////////////////////////

    // std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
    out1.substract_mean_normalize(mean_vals_inv, norm_vals_inv);
    out2.substract_mean_normalize(mean_vals_inv, norm_vals_inv);

  //memcpy((uchar*)score.data, out1.data, sizeof(float) * out1.w * out1.h);
    out1.to_pixels(score.data, ncnn::Mat::PIXEL_GRAY);
    out2.to_pixels(desc.data, ncnn::Mat::PIXEL_BGR);

    auto fea_pts_vec =  tracker.extractFeature(score);
    // std::cout << "fea_size : "<<fea_pts_vec.size() << std::endl;
    
    return fea_pts_vec;
 }

  void ros2cv(sensor_msgs::CompressedImageConstPtr &image_ros,cv::Mat &image,cv::Mat &intrisic,cv::Mat &distCoeffs,bool corr)
  {
    // ROS_Image -> Openshow_image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_ros, sensor_msgs::image_encodings::RGB8);
    image = cv_ptr->image;
    // std::cout << image.size() << std::endl;
    if (image.cols == 640 || image.cols == 1280)
    {
      cv::resize(image, image, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
    }

    if (corr)
    {
      /* code */
      // 图像去畸变
      cv::Mat map1, map2;
      cv::Size imageSize = image.size();
      cv::initUndistortRectifyMap(intrisic, distCoeffs, cv::Mat(), cv::getOptimalNewCameraMatrix(intrisic, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
      cv::remap(image, image, map1, map2, cv::INTER_LINEAR); // correct the distortion

    }
    
    // return;
    };
  }; 
//namespce cocolic

