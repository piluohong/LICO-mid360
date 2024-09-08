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

#pragma once

#include <ros/ros.h>

#include <odom/msg_manager.h>
#include <odom/odometry_viewer.h>
#include <odom/trajectory_manager.h>

#include <imu/imu_state_estimator.h>
#include <imu/imu_initializer.h>
#include <lidar/lidar_handler.h>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <camera/r3live.hpp>
#include <ncnn_node/ncnn_image.hpp>

namespace cocolic
{

  enum KnotDensity
  {
    gear1 = 1, // 0.1
    gear2 = 2, // 0.05
    gear3 = 3, // 0.033
    gear4 = 4  // 0.025
  };
  class OdometryManager
  {
  public:
    
    OdometryManager(const YAML::Node &node, ros::NodeHandle &nh);

    void RunBag();

    void RunInSubscribeMode();

    double SaveOdometry();

    std::vector<int> cp_num_vec;

    int GetKnotDensity(double gyro_norm, double acce_norm)
    {
      if (gyro_norm < 0.0 || acce_norm < 0.0)
      {
        std::cout << RED << "gyro_norm/acce_norm is wrong!" << RESET << std::endl;
      }

      int gyro_density = -1, acce_density = -1;

      acce_norm = std::abs(acce_norm - gravity_norm_);
      LOG(INFO) << "[acce_norm] " << acce_norm;
      if (acce_norm < 0.5)
      { // [0, 0.5)
        acce_density = KnotDensity::gear1;
      }
      else if (acce_norm < 1.0)
      { // [0.5, 1.0)
        acce_density = KnotDensity::gear2;
      }
      else if (acce_norm < 5.0)
      { // [1.0, 5.0)
        acce_density = KnotDensity::gear3;
      }
      else
      { // [5.0, -)
        acce_density = KnotDensity::gear4;
      }

      LOG(INFO) << "[gyro_norm] " << gyro_norm;
      if (gyro_norm < 0.5)
      { // [0, 0.5)
        gyro_density = KnotDensity::gear1;
      }
      else if (gyro_norm < 1.0)
      { // [0.5, 1.0)
        gyro_density = KnotDensity::gear2;
      }
      else if (gyro_norm < 5.0)
      { // [1.0, 5.0)
        gyro_density = KnotDensity::gear3;
      }
      else
      { // [5.0, -)
        gyro_density = KnotDensity::gear4;
      }

      return std::max(gyro_density, acce_density);
    };

    inline float pointDistance(Eigen::Vector3f p)
    {
        return sqrt(p(0)*p(0) + p(1)*p(1) + p(2)*p(2));
    }

    inline void getColor(float p, float np, float&r, float&g, float&b) 
    {
            float inc = 6.0 / np;
            float x = p * inc;
            r = 0.0f; g = 0.0f; b = 0.0f;
            if ((0 <= x && x <= 1) || (5 <= x && x <= 6)) r = 1.0f;
            else if (4 <= x && x <= 5) r = x - 4;
            else if (1 <= x && x <= 2) r = 1.0f - (x - 1);

            if (1 <= x && x <= 3) g = 1.0f;
            else if (0 <= x && x <= 1) g = x - 0;
            else if (3 <= x && x <= 4) g = 1.0f - (x - 3);

            if (3 <= x && x <= 5) b = 1.0f;
            else if (2 <= x && x <= 3) b = x - 2;
            else if (5 <= x && x <= 6) b = 1.0f - (x - 5);
            r *= 255.0;
            g *= 255.0;
            b *= 255.0;
    }

  protected:
    bool CreateCacheFolder(const std::string &config_path,
                           const std::string &bag_path);

    void SolveLICO();

    void ProcessLICData();

    void ProcessImageData();

    bool PrepareTwoSegMsgs(int seg_idx);

    void UpdateTwoSeg();

    bool PrepareMsgs();

    void UpdateOneSeg();

    void SetInitialState();

    void PublishCloudAndTrajectory();

  protected:
    OdometryMode odometry_mode_;

    MsgManager::Ptr msg_manager_;

    bool is_initialized_;
    IMUInitializer::Ptr imu_initializer_;

    Trajectory::Ptr trajectory_;
    TrajectoryManager::Ptr trajectory_manager_;

    LidarHandler::Ptr lidar_handler_;

    R3LIVE::Ptr camera_handler_;
    ncnn_image ncnn_handler_;

    int64_t t_begin_add_cam_; // 

    OdometryViewer odom_viewer_;

    int update_every_k_knot_;

    /// [nurbs]
    double t_add_;
    int64_t t_add_ns_;
    bool non_uniform_;
    double distance0_;

    int cp_add_num_coarse_;
    int cp_add_num_refine_;

    int lidar_iter_;
    bool use_lidar_scale_;

    std::string cache_path_;

    double pasue_time_;

    TimeStatistics time_summary_;

    struct SysTimeOffset
    {
      SysTimeOffset(double t1, double t2, double t3, double t4)
          : timestamp(t1), t_lidar(t2), t_cam(t3), t_imu(t4) {}
      double timestamp = 0;
      double t_lidar = 0;
      double t_cam = 0;
      double t_imu = 0;
    };
    std::vector<SysTimeOffset> sys_t_offset_vec_;

  private:
    double gravity_norm_;

    int64_t traj_max_time_ns_cur;
    int64_t traj_max_time_ns_next;
    int64_t traj_max_time_ns_next_next;

    int cp_add_num_cur;
    int cp_add_num_next;
    int cp_add_num_next_next;

    bool is_evo_viral_;

    double ave_r_thresh_;
    double ave_a_thresh_;

    VPointCloud sub_map_cur_frame_point_;

    Eigen::aligned_vector<Eigen::Vector3d> v_points_;
    Eigen::aligned_vector<Eigen::Vector2d> px_obss_;

    Eigen::Matrix3d K_;

    bool ikdtree_debug = false;
  };

} // namespace cocolic
