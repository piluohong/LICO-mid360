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

#include <cocolic/feature_cloud.h>
#include <lidar/lidar_feature.h>
#include <spline/trajectory.h>
#include <utils/cloud_tool.h>
#include <utils/yaml_utils.h>
#include <ikd-tree/ikd_Tree.h>
#include <ivox3d/ivox3d.h>


namespace cocolic
{

  class LidarHandler
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<LidarHandler> Ptr;

    LidarHandler(const YAML::Node &node, Trajectory::Ptr traj);

    // 
    void FeatureCloudHandler(const cocolic::feature_cloud::ConstPtr &feature_msg);

    // 
    void FeatureCloudHandler(const LiDARFeature &lidar_feature);

    void FeatureCloudHandler(int64_t scan_timestamp, int64_t scan_time_max,
                             const RTPointCloud::Ptr corner_feature,
                             const RTPointCloud::Ptr surface_feature,
                             const RTPointCloud::Ptr raw_cloud);
    // 
    bool UpdateLidarSubMap();

    void SetUpdateMap(bool update_map) { key_frame_updated_ = update_map; }

    int64_t GetLatestFeatureTime()
    {
      return latest_feature_time_;
    }

    PosCloud::Ptr GetLatestSurfaceFeature()
    {
      if (local_feature_container_.find(latest_feature_time_) == local_feature_container_.end())
        return nullptr;
      return local_feature_container_[latest_feature_time_].surface_features;
    }

    std::pair<PosCloud::Ptr, double> GetLatestSurfaceFeatureBeforeActiveTime(int64_t active_time)
    {
      int latest_idx = INT_MAX;
      for (int i = feature_cur_vec_.size() - 1; i >=0; i--)
      {
        if (feature_cur_vec_[i].time_max < active_time)
        {
          latest_idx = i;
          break;
        }
      }

      PosCloud::Ptr cloud_distort(new PosCloud);
      cloud_distort->points.clear();
      double cloud_time = -1;

      if (latest_idx == INT_MAX)
      {
        std::cout << "[GetLatestSurfaceFeatureBeforeActiveTime fail]\n";
      }
      else
      {
        cloud_distort = feature_cur_vec_[latest_idx].surface_features;
        cloud_time = feature_cur_vec_[latest_idx].timestamp;
        feature_cur_vec_.erase(feature_cur_vec_.begin() + latest_idx);
      }

      return std::make_pair(cloud_distort, cloud_time);
    }

    LiDARFeature GetLatestLidarFeatureBeforeActiveTime(int64_t active_time)
    {
      int latest_idx = INT_MAX;
      for (int i = feature_cur_vec_.size() - 1; i >=0; i--)
      {
        if (feature_cur_vec_[i].time_max < active_time)
        {
          latest_idx = i;
          break;
        }
      }

      LiDARFeature lidar_feature;
      if (latest_idx == INT_MAX)
      {
        std::cout << "[GetLatestSurfaceFeatureBeforeActiveTime fail]\n";
      }
      else
      {
        lidar_feature = feature_cur_vec_[latest_idx];
        feature_cur_vec_.erase(feature_cur_vec_.begin() + latest_idx);
      }

      return lidar_feature;
    }

    // 
    void GetLoamFeatureAssociation();

    // 
    const Eigen::aligned_vector<PointCorrespondence> &GetPointCorrespondence()
        const
    {
      return point_correspondence_ds_;
    }

    // loop closure
    PosCloud::Ptr GetCloudKeyPos2D() const { return cloud_key_pos_xy_; };

    const PosCloud::Ptr GetCloudKeyPos3D() const { return cloud_key_pos_; };

    void FindNearbyKeyFrames(const int key, const int search_num,
                             PosCloud::Ptr &cloud_in_G) const;

    void UpdateCloudKeyPos(const std::pair<int, int> &cur_wrt_history);

    // 
    const LiDARFeature &GetFeatureCurrent() const { return feature_cur_; }

    const LiDARFeature &GetFeatureCurrentDs() const { return feature_cur_ds_; }

    const LiDARFeature &GetFeatureMap() const { return feature_map_; }

    const LiDARFeature &GetFeatureMapDs() const { return feature_map_ds_; }

    VPointCloud map_corrs_viewer;

     // 更新localmap的边界
    void localmap_fov_segment();
    // 初始化ikdtree
    bool initial_ikdtree();
    // 更新localmap
    void localmap_incremental(int64_t &Newtimestamp);
    void localmap_incremental(int64_t &Newtimestamp, bool ivox);
    inline float point_dist(PosPoint p1, PosPoint p2)
    {
      float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
      return d;
    }

    inline void set_ivox_option()
    {
        if (ivox_nearby_type == 0) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
        } else if (ivox_nearby_type == 6) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
        } else if (ivox_nearby_type == 18) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        } else if (ivox_nearby_type == 26) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
        } else {
            std::cout << "unknown ivox_nearby_type, use NEARBY18\n";
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        }

        ivox_options_.resolution_ = 0.5; // default: 0.2

        // init ivox
        ivox_ = std::make_shared<IVoxType>(ivox_options_);
        first_scan_flg = false;

    }
   

  private:
    bool UpdateKeyFrames();

    const bool IsKeyFrame(int64_t time) const;

    // 
    void GetNearDistKeyScanID(PosCloud::Ptr key_pos_selected);

    // 
    void GetNearTimeKeyScanID(PosCloud::Ptr key_pos_selected,
                              const int64_t cur_time) const;

    void TransformLiDARFeature(const LiDARFeature &lf_in,
                               const Eigen::Matrix4d &T_in_to_out,
                               LiDARFeature &lf_out) const;

    void DownsampleLiDARFeature(const LiDARFeature &lf_in,
                                LiDARFeature &lf_out) const;

    // 
    void ExtractSurroundFeatures(const int64_t cur_time);

    void SetTargetMap(const LiDARFeature &feature_map);

    /// target : feature_map_ds
    bool FindCorrespondence(const LiDARFeature &lf_cur,
                            const LiDARFeature &lf_cur_in_M);

    void DownSampleCorrespondence();

   

  private:
    Trajectory::Ptr trajectory_;

    bool use_corner_feature_;

    /// current scan
    std::vector<LiDARFeature> feature_cur_vec_;
    LiDARFeature feature_cur_;
    LiDARFeature feature_cur_ds_;
    LiDARFeature feature_cur_undis_;
    LiDARFeature feature_cur_ds_undis_;
    LiDARFeature feature_cur_ds_world;
    LiDARFeature featsFromMap;
    int edge_min_valid_num_;
    int surf_min_valid_num_;
    float corner_leaf_size_;
    float surface_leaf_size_;

    /// 
    Eigen::aligned_vector<PointCorrespondence> point_correspondence_;
    Eigen::aligned_vector<PointCorrespondence> point_correspondence_ds_;
    /// 
    int cor_downsample_;

    bool update_full_cloud_;
    /// map
    LiDARFeature feature_map_;    // full_cloud is set by update_full_cloud_
    LiDARFeature feature_map_ds_; // full_cloud is empty
    pcl::KdTreeFLANN<PosPoint>::Ptr kdtree_corner_map_;
    pcl::KdTreeFLANN<PosPoint>::Ptr kdtree_surface_map_;
    float keyframe_search_radius_;
    int64_t keyframe_search_time_;
    float keyframe_density_map_;

    // 
    int64_t latest_feature_time_;

    /// 
    std::map<int64_t, LiDARFeature> local_feature_container_;
    /// 
    std::map<int64_t, LiDARFeature> cache_feature_container_; // 
    /// 
    PosCloud::Ptr cloud_key_pos_;

    std::map<int64_t, LiDARFeature> local_feature_container_all_ds_;

    // for loop closure
    PosCloud::Ptr cloud_key_pos_xy_;
    bool key_frame_updated_;

    /// 
    double keyframe_angle_degree_;
    double keyframe_dist_meter_;
    int64_t keyframe_time_second_;

    int64_t cloud_reserved_time_;

  private:
    //use ikd_tree in local map
    // bool use_ikdtree_flg = true;
    bool first_initial_ikdtree;
    KD_TREE<PosPoint> ikdtree;
    vector<BoxPointType> cub_needrm;
    vector<KD_TREE<PosPoint>::PointVector> Nearest_Points;
    
    int kdtree_delete_counter = 0;
    BoxPointType LocalMap_Points; //ikd-tree地图立方体的2个角点
    bool Localmap_Initialized = false;
    double cube_len = 2000.0;
    const float MOV_THRESHOLD = 1.5f;
    float DET_RANGE = 300.0f;

    uint16_t feature_cur_ds_size;
    int add_point_size;

    const int NUM_MATCH_POINTS = 5;
    const float filter_size_map_min = 0.5;

   

    // use iVox in local map. 
  public:
  #ifdef IVOX_NODE_TYPE_PHC
      using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PosPoint>; // 伪希尔伯特空间填充曲线进行查询近邻点
  #else
      using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PosPoint>;
  #endif

    bool use_ivox_ = false;

  private:
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;

    const double filter_size_map_min_ = 0.5; // livox: 0.: faster-lio中倾向于加入scan_undistort中所有点
    const int ivox_nearby_type = 18; // default: 16 18 26
   
    // float det_range_ = 300.0f;
    // double cube_len_ = 0;
    // bool localmap_initialized_ = false;
    bool first_scan_flg = true;

  };

} // namespace cocolic
