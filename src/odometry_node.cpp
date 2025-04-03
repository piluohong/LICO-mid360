/*
 * @Author: piluohong 1912694135@qq.com
 * @Date: 2024-05-27 22:48:25
 * @LastEditors: piluohong 1912694135@qq.com
 * @LastEditTime: 2025-04-03 20:18:54
 * @FilePath: /lvio/src/Coco-LIC-master/src/odometry_node.cpp
 * @Description: 
 * */

/*
 * Coco-LIC: Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <csignal>
#include <ros/package.h>
#include <ros/ros.h>
#include <odom/odometry_manager.h>

using namespace cocolic;

bool flg_exit = false;
void SigHandle(int sig) {
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "cocolic");
  ros::NodeHandle nh("~");

  std::string config_path;
  nh.param<std::string>("config_path", config_path, "ct_odometry.yaml");
  ROS_INFO("Odometry load %s.", config_path.c_str());

  YAML::Node config_node = YAML::LoadFile(config_path);
  

  std::string log_path = config_node["log_path"].as<std::string>();
  FLAGS_log_dir = log_path;
  FLAGS_colorlogtostderr = true;
  std::cout << "\n🥥 Start LICO(mid360) Odometry 🥥";

  OdometryManager odom_manager(config_node, nh);// 初始化數據讀入
  MODE mode = MODE(config_node["mode"].as<int>());
  
  signal(SIGINT, SigHandle);
  if (MODE::Odometry_Offline == mode) {
    // float t0 = omp_get_wtime();
    odom_manager.RunBag();
    // float t1 = omp_get_wtime();
    // odom_manager.time_vec.push_back(t1-t0);
  } else
  {
      
      ros::Rate rate(5000);
      // std::cout  << "RunInSubscribeMode ...\n.";
      while(ros::ok()){
        if (flg_exit) {
            break;
        }
        // float t0 = omp_get_wtime();
        odom_manager.RunInSubscribeMode();
        // float t1 = omp_get_wtime();
        // odom_manager.time_vec.push_back(t1-t0);
        ros::spinOnce();
        rate.sleep();
    }
  }
  // odom_manager.saveGlobalmap();
  odom_manager.savekntsNum("/home/hyq/slam/lvio/src/LICO-mid360-main/data/adaptive_kntsNum.txt");
  odom_manager.saveCosttime("/home/hyq/slam/lvio/src/LICO-mid360-main/data/adaptive_timecost.txt");
  // double t_traj_max = odom_manager.SaveOdometry();
  auto s_vec = odom_manager.pose_final.front().translation();
  auto e_vec = odom_manager.pose_final.back().translation();
  float err_s2e = std::sqrt(std::pow((e_vec[0] - s_vec[0]),2) + std::pow((e_vec[1] - s_vec[1]),2) + std::pow((e_vec[2] - s_vec[2]),2));
  std::cout <<"End-to-end error : " << err_s2e << " m" << std::endl;
  std::cout << "Saving how many frames : " << odom_manager.all_globalmap.size() << std::endl;
  std::cout << "\n✨ All Done.\n\n";
  return 0;
}
