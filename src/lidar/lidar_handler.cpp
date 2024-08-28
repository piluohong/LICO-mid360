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

#include <pcl/kdtree/impl/kdtree_flann.hpp> // pcl::KdTreeFLANN

#include <lidar/lidar_handler.h>
// #include <lidar/livox_feature_extraction.h>
#include <lidar/velodyne_feature_extraction.h>
#include <pcl/common/transforms.h>           //pcl::transformPointCloud
#include <pcl_conversions/pcl_conversions.h> // pcl::fromROSMsg

#include <algorithm> // std::find



namespace cocolic
{

  struct by_timestamp
  {
    bool operator()(const PointCorrespondence &left,
                    const PointCorrespondence &right)
    {
      return left.t_point < right.t_point;
    }
  };

  LidarHandler::LidarHandler(const YAML::Node &node, Trajectory::Ptr traj)
      : trajectory_(std::move(traj)),
        use_corner_feature_(true),
        cor_downsample_(1),
        update_full_cloud_(false),
        kdtree_corner_map_(new pcl::KdTreeFLANN<PosPoint>()),
        kdtree_surface_map_(new pcl::KdTreeFLANN<PosPoint>()),
        latest_feature_time_(-1),
        cloud_key_pos_(new PosCloud),
        cloud_key_pos_xy_(new PosCloud),
        key_frame_updated_(false)
  {
    use_corner_feature_ = yaml::GetBool(node, "use_corner_feature", false);

    const YAML::Node &map_node = node["map_param"];
    keyframe_search_radius_ = map_node["keyframe_search_radius"].as<float>();
    keyframe_search_time_ = map_node["keyframe_search_time"].as<float>() * S_TO_NS;
    keyframe_density_map_ = map_node["keyframe_density"].as<float>();

    cloud_reserved_time_ =
        yaml::GetValue<double>(map_node, "cloud_reserved_time", -1) * S_TO_NS;

    const YAML::Node &cur_scan_node = node["current_scan_param"];
    edge_min_valid_num_ = cur_scan_node["edge_min_valid_num"].as<int>();
    surf_min_valid_num_ = cur_scan_node["surf_min_valid_num"].as<int>();
    corner_leaf_size_ = cur_scan_node["corner_leaf_size"].as<float>();
    surface_leaf_size_ = cur_scan_node["surface_leaf_size"].as<float>();
    cor_downsample_ = cur_scan_node["correspondence_downsample"].as<int>();

    const YAML::Node &keyframe_node = node["keyframe_strategy"];
    keyframe_angle_degree_ = keyframe_node["angle_degree"].as<double>();
    keyframe_dist_meter_ = keyframe_node["dist_meter"].as<double>();
    keyframe_time_second_ = keyframe_node["time_second"].as<double>() * S_TO_NS;

    feature_cur_vec_.clear();
  }

  // Note 
  // void LidarHandler::FeatureCloudHandler(
  //     const cocolic::feature_cloud::ConstPtr &feature_msg)
  // {
  //   RTPointCloud::Ptr corner_cloud(new RTPointCloud);
  //   RTPointCloud::Ptr surface_cloud(new RTPointCloud);
  //   RTPointCloud::Ptr full_cloud(new RTPointCloud);

  //   pcl::fromROSMsg(feature_msg->corner_cloud, *corner_cloud);
  //   pcl::fromROSMsg(feature_msg->surface_cloud, *surface_cloud);
  //   pcl::fromROSMsg(feature_msg->full_cloud, *full_cloud);

  //   feature_cur_.Clear();

  //   double scan_time = feature_msg->header.stamp.toSec();
  //   double traj_start_time = trajectory_->GetDataStartTime();
  //   feature_cur_.timestamp = scan_time - traj_start_time;
  //   CloudToRelativeMeasureTime(corner_cloud, scan_time, traj_start_time);
  //   CloudToRelativeMeasureTime(surface_cloud, scan_time, traj_start_time);
  //   CloudToRelativeMeasureTime(full_cloud, scan_time, traj_start_time);

  //   if (use_corner_feature_)
  //   {
  //     pcl::RTPointCloudToPosCloud(corner_cloud, feature_cur_.corner_features);
  //   }
  //   pcl::RTPointCloudToPosCloud(surface_cloud, feature_cur_.surface_features);
  //   pcl::RTPointCloudToPosCloud(full_cloud, feature_cur_.full_cloud,
  //                               &feature_cur_.time_max);
  //   //
  //   DownsampleLiDARFeature(feature_cur_, feature_cur_ds_);
  // }

  // // Note 
  // void LidarHandler::FeatureCloudHandler(const LiDARFeature &lidar_feature)
  // {
  //   feature_cur_.Clear();
  //   feature_cur_.time_max = lidar_feature.time_max;
  //   feature_cur_.timestamp = lidar_feature.timestamp;
  //   *(feature_cur_.corner_features) = *(lidar_feature.corner_features);
  //   *(feature_cur_.surface_features) = *(lidar_feature.surface_features);
  //   *(feature_cur_.full_cloud) = *(lidar_feature.full_cloud);

  //   /// 
  //   DownsampleLiDARFeature(feature_cur_, feature_cur_ds_);
  // }

  // Note 当前使用
  void LidarHandler::FeatureCloudHandler(int64_t scan_timestamp,
                                          int64_t scan_time_max,
                                          const RTPointCloud::Ptr corner_feature,
                                          const RTPointCloud::Ptr surface_feature,
                                          const RTPointCloud::Ptr raw_cloud)
  {
    feature_cur_.Clear();
    feature_cur_.timestamp = scan_timestamp;
    feature_cur_.time_max = scan_time_max;
    if (use_corner_feature_)
    {
      pcl::RTPointCloudToPosCloud(corner_feature, feature_cur_.corner_features);
    }
    if (surface_feature->empty()) ROS_WARN("surface_feature error\n");
    pcl::RTPointCloudToPosCloud(surface_feature, feature_cur_.surface_features);
    if (raw_cloud->empty()) ROS_WARN("full_cloud error\n");
    pcl::RTPointCloudToPosCloud(raw_cloud, feature_cur_.full_cloud);

    // feature_cur_vec_.push_back(feature_cur_);

    // downsample the current lidar feature
    
    DownsampleLiDARFeature(feature_cur_, feature_cur_ds_);
  }

  bool LidarHandler::UpdateLidarSubMap()
  {
    // 关键帧更新 -> update cloud_key_pos_ & local_feature_container_ & local_feature_container_all_ds_
    std::cout << "----------------------------------------------------------------------------\n";
    // DownsampleLiDARFeature(feature_cur_, feature_cur_ds_);
    feature_cur_ds_size = feature_cur_ds_.surface_features->size();
    // std::cout << "Corner Points size : " << feature_cur_.corner_features->size() << std::endl; // 0
    // std::cout << "Surface Points size : " << feature_cur_.surface_features->size() << std::endl;
    // std::cout << "Full Points size : " << feature_cur_.full_cloud->size() << std::endl;
    // std::cout << "DS Corner Points size : " << feature_cur_ds_.corner_features->size() << std::endl;
    // std::cout << "DS Surface Points size : " << feature_cur_ds_.surface_features->size() << std::endl;
    // std::cout << "DS Full Points size : " << feature_cur_ds_.full_cloud->size() << std::endl;
    
     // 更新localmap边界
    localmap_fov_segment();
    // initial ikdtree -> true; no first scan intial -> false
    first_initial_ikdtree = initial_ikdtree();
    
    std::cout << "(int)first_initial_ikdtree : " << (int)first_initial_ikdtree << std::endl;
    if (first_initial_ikdtree)
    {
      ROS_WARN("Only execute on first scan !!!! \n");
      UpdateKeyFrames();
      if (key_frame_updated_)
      {
        
        key_frame_updated_ = false; // 重置关键帧更新标志
        ExtractSurroundFeatures(feature_cur_.timestamp); // 提取特征子图，执行完，特征子图的kdtree已建立
        return true;
      }
      else
      {
        
          return false; //不是关键帧
      }
      
    }
    else
    {
      // // debug
      // feature_cur_ds_undis_.timestamp = feature_cur_ds_.timestamp;
      // feature_cur_ds_undis_.time_max = feature_cur_ds_.time_max;
      //   trajectory_->UndistortScan(*(feature_cur_ds_.corner_features),
      //                              feature_cur_ds_.timestamp,
      //                              *(feature_cur_ds_undis_.corner_features));
      //   trajectory_->UndistortScan(*(feature_cur_ds_.surface_features),
      //                              feature_cur_ds_.timestamp,
      //                              *(feature_cur_ds_undis_.surface_features));
      //   trajectory_->UndistortScan(*(feature_cur_ds_.full_cloud), 
      //                               feature_cur_ds_.timestamp,
      //                              *(feature_cur_ds_undis_.full_cloud));
      // LiDARFeature debug_world_1;
      // SE3d cur_pos = trajectory_->GetLidarPoseNURBS(feature_cur_.timestamp);
      // TransformLiDARFeature(feature_cur_ds_undis_,
      //                       cur_pos.matrix(), debug_world_1);


      // LiDARFeature debug_world_2;
      // trajectory_->UndistortScanInG(*feature_cur_ds_.surface_features,
      //                             feature_cur_ds_.timestamp,
      //                             *debug_world_2.surface_features);
      
      
      // std::cout << debug_world_1.surface_features->points[1000].x << ", " << debug_world_2.surface_features->points[1000].x << std::endl;
    }

    return false;
  }

  void LidarHandler::localmap_fov_segment()
  {
    // if (use_ikdtree_flg)
    // {

      // static int first_flg = 0;
      cub_needrm.clear();
      kdtree_delete_counter = 0;

      // W系下位置
      Eigen::Vector3d p =
            trajectory_->GetLidarPoseNURBS(feature_cur_.timestamp).translation();
      // 初始化局部地图范围， 以pos_LiD为中心，长宽高均为cube_len
      if (!Localmap_Initialized)
      {
        for (int i = 0; i < 3; i++)
        {
          LocalMap_Points.vertex_min[i] = p(i) - cube_len / 2.0;
          LocalMap_Points.vertex_max[i] = p(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
      }

      //各个方向上p与局部地图边界的距离
      float dist_to_map_edge[3][2];
      bool need_move = false;
      for (int i = 0; i < 3; i++)
      {
        dist_to_map_edge[i][0] = fabs(p(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(p(i) - LocalMap_Points.vertex_max[i]);
        // 与某个方向上的边界距离(1.5*300)太小，标记需要移除need_move
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
          need_move = true;
      }

      if(!need_move)
        return;

      BoxPointType New_LocalMap_Points, tem_boxpoints;
      New_LocalMap_Points = LocalMap_Points;

      // 需要移动的距离
      float mov_dist = std::max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
      for (int i = 0; i < 3; i++)
      {
        tem_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
          New_LocalMap_Points.vertex_max[i] -= mov_dist;
          New_LocalMap_Points.vertex_min[i] -= mov_dist;
          tem_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
          cub_needrm.push_back(tem_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        { 
          New_LocalMap_Points.vertex_max[i] += mov_dist;
          New_LocalMap_Points.vertex_min[i] += mov_dist;
          tem_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
          cub_needrm.push_back(tem_boxpoints);
        }
      }
      LocalMap_Points = New_LocalMap_Points;

      KD_TREE<PosPoint>::PointVector points_history;
      ikdtree.acquire_removed_points(points_history);

      if (cub_needrm.size() > 0) 
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);

    // }

  }

  bool LidarHandler::initial_ikdtree()
  {
      
      std::cout << " Cur feats scan's size : " << feature_cur_ds_size << std::endl;
      if (feature_cur_ds_size < 5)
      {
          ROS_WARN("No point, skip this scan!\n");
          return true;
      }
    
    // 执行一次
    if (ikdtree.Root_Node == nullptr)
    {
      ikdtree.set_downsample_param(0.5);
      LiDARFeature debug_world;
      // feature_cur_ds_world.surface_features->resize(feature_cur_ds_size);
      // UndistortScan && UndistortScanInG
      trajectory_->UndistortScanInG(*feature_cur_ds_.surface_features,
                                  feature_cur_ds_.timestamp,
                                  *feature_cur_ds_world.surface_features);
      
      // debug -> different result
      // trajectory_->UndistortScan(*feature_cur_ds_.surface_features,
      //                             feature_cur_ds_.timestamp,
      //                             *debug_world.surface_features);
      
      // debug -> same result
      // SE3d pose_cur_to_G = trajectory_->GetLidarPoseNURBS(feature_cur_ds_.timestamp); // TG_Lcur  //TODO：    
      // TransformLiDARFeature(feature_cur_ds_,
      //                       pose_cur_to_G.matrix(), debug_world);

      // std::cout << feature_cur_ds_world.surface_features->points[100].x << ", " << debug_world.surface_features->points[100].x << std::endl;
      ikdtree.Build(feature_cur_ds_world.surface_features->points);
      ROS_WARN("initial ikdtree successfully! \n");
      return true;
    }

    
    return false;
  }

  void LidarHandler::localmap_incremental(int64_t &Newtimestamp)
  {
      if (!first_initial_ikdtree)
      {
        int64_t time = Newtimestamp;
        float  filter_size_map_min = 0.5;
        // add feature_cur_ds_.surface_features to localmap
        feature_cur_ds_world.surface_features->resize(feature_cur_ds_size);
        KD_TREE<PosPoint>::PointVector PointToAdd;
        KD_TREE<PosPoint>::PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feature_cur_ds_size);
        PointNoNeedDownsample.reserve(feature_cur_ds_size);
        // 取最新的位姿估计结果更新当前帧点云
        trajectory_->UndistortScanInG(*feature_cur_ds_.surface_features ,time , *feature_cur_ds_world.surface_features);
        
        for (int i = 0; i < feature_cur_ds_size; i++)
        {
          if (!Nearest_Points[i].empty())
          {
            const KD_TREE<PosPoint>::PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PosPoint mid_point;
            mid_point.x = floor(feature_cur_ds_world.surface_features->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feature_cur_ds_world.surface_features->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feature_cur_ds_world.surface_features->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = point_dist(feature_cur_ds_world.surface_features->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * 0.5 && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min 
                && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feature_cur_ds_world.surface_features->points[i]);
                continue;
            }
            for (int j = 0; j < NUM_MATCH_POINTS; j++)
            {
              if (points_near.size() < NUM_MATCH_POINTS)
                break;
              if (point_dist(points_near[j], mid_point) < dist)
              {
                need_add = false;
                break;
              }
            }
            if (need_add)
              PointToAdd.push_back(feature_cur_ds_world.surface_features->points[i]);
          }
          else
          {
              PointToAdd.push_back(feature_cur_ds_world.surface_features->points[i]);
          } 
        }

        add_point_size = ikdtree.Add_Points(PointToAdd,true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
        add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();

        KD_TREE<PosPoint>::PointVector().swap(ikdtree.PCL_Storage);
        ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
        featsFromMap.timestamp = feature_cur_ds_.timestamp;
        featsFromMap.time_max = feature_cur_ds_.time_max;
        featsFromMap.surface_features->clear();
        featsFromMap.surface_features->points = ikdtree.PCL_Storage;
        feature_map_ds_ = featsFromMap;
      }
       
       first_initial_ikdtree = false;
       std::cout << "After update ikdtree's size :" << featsFromMap.surface_features->size() << std::endl;
  }

  

  void LidarHandler::GetLoamFeatureAssociation()
  {
    Nearest_Points.resize(feature_cur_ds_.surface_features->size()); // 设置当前帧的临近点容器

    LiDARFeature cur_lf_in_map;

    if (use_corner_feature_)
    {
      trajectory_->UndistortScanInG(*feature_cur_ds_.corner_features,
                                    feature_cur_ds_.timestamp,
                                    *cur_lf_in_map.corner_features);
    }

    trajectory_->UndistortScanInG(*feature_cur_ds_.surface_features,
                                  feature_cur_ds_.timestamp,
                                  *cur_lf_in_map.surface_features);
    // std::cout << feature_cur_ds_.surface_features->points[10].x << ", " <<   cur_lf_in_map.surface_features->points[10].x << std::endl;
    // 关联特征点对
    FindCorrespondence(feature_cur_ds_, cur_lf_in_map);
    std::sort(point_correspondence_.begin(), point_correspondence_.end(),
              by_timestamp());

    DownSampleCorrespondence();

    LOG(INFO) << "point correspondence : " << point_correspondence_.size()
              << "; point correspondence_ds : "
              << point_correspondence_ds_.size();
  }

  bool LidarHandler::UpdateKeyFrames()
  {
    // 
    // undistort points
    // static int cnt = 0;
   
    // std::cout << "DS Corner Points size : " << feature_cur_ds_.corner_features->size() << std::endl;
    // std::cout << "DS Surface Points size : " << feature_cur_ds_.surface_features->size() << std::endl;
    // std::cout << "DS Full Points size : " << feature_cur_ds_.full_cloud->size() << std::endl;
    // feature_cur_ds_.corner_features->clear();
    // feature_cur_ds_.surface_features->clear();
    // feature_cur_ds_.full_cloud->clear();
    // if(use_ikdtree_flg)
    // {
     // return false;
    // }


    LiDARFeature feature;
    feature = feature_cur_;
    cache_feature_container_[feature.timestamp] = feature;
    // ROS_INFO(">>> cache_fea_container's size : %d\n", cache_feature_container_.size());

    // 
    for (auto iter = cache_feature_container_.begin();
         iter != cache_feature_container_.end();)
    {
      // [Case 1] lio-sam中的方式判断是否为关键帧
      if (!IsKeyFrame(iter->second.time_max))
      {
        // 如果不是关键帧
        if (iter->second.time_max >=
            trajectory_->GetActiveTime())
        { 
          break;
        }else
        {
          cache_feature_container_.erase(iter++); // 删除当前非关键帧元素，并更新iter
          continue;
        }
      }
      
      /// // 如果是关键帧执行 [Case 2]
      LiDARFeature key_scan;
      key_scan.timestamp = iter->second.timestamp;
      key_scan.time_max = iter->second.time_max;
      key_scan = iter->second;
      if (local_feature_container_.empty())
      {
        // 
        // key_pose.timestamp = trajectory_->minTime(LiDARSensor);
        *(key_scan.corner_features) = *(iter->second.corner_features);
        *(key_scan.surface_features) = *(iter->second.surface_features);
        *(key_scan.full_cloud) = *(iter->second.full_cloud);
      }
      else
      {
        // 
        trajectory_->UndistortScan(*(iter->second.corner_features),
                                   key_scan.timestamp,
                                   *(key_scan.corner_features));
        trajectory_->UndistortScan(*(iter->second.surface_features),
                                   key_scan.timestamp,
                                   *(key_scan.surface_features));
        trajectory_->UndistortScan(*(iter->second.full_cloud), key_scan.timestamp,
                                   *(key_scan.full_cloud));

        
      }
#if false //编译时候的屏蔽选项
    if (local_feature_container_.size() < 10)
      local_feature_container_[key_scan.timestamp] = key_scan;
    else {
      LiDARFeature key_scan_ds;
      DownsampleLiDARFeature(key_scan, key_scan_ds);
      local_feature_container_[key_scan.timestamp] = key_scan_ds;
    }
#else
      LiDARFeature key_scan_ds;
      DownsampleLiDARFeature(key_scan, key_scan_ds);
      local_feature_container_[key_scan.timestamp] = key_scan;
      local_feature_container_all_ds_[key_scan.timestamp] = key_scan_ds;
      // 
      latest_feature_time_ = key_scan.timestamp;
#endif
      cache_feature_container_.erase(iter++);

      PosPoint key_pose;
      key_pose.timestamp = key_scan.timestamp;
      Eigen::Vector3d p =
          trajectory_->GetLidarPoseNURBS(key_scan.timestamp).translation();
      key_pose.x = p[0];
      key_pose.y = p[1];
      key_pose.z = p[2];
      cloud_key_pos_->push_back(key_pose);

      key_pose.z = 0;
      cloud_key_pos_xy_->push_back(key_pose);

      // trajectory_->SetForcedFixedTime(key_scan.time_max);

      key_frame_updated_ = true;
    }
    // if (key_frame_updated_)
      // ROS_WARN(">>> Curr is key pose\n");

    // 不执行
    // if (cloud_reserved_time_ > 0)
    // {
    //   double t_now = local_feature_container_.rbegin()->first;
    //   auto iter = local_feature_container_.begin();
    //   while (iter != local_feature_container_.end())
    //   {
    //     if (t_now - (*iter).first > cloud_reserved_time_)
    //     {
    //       iter = local_feature_container_.erase(iter);
    //     }
    //     else
    //     {
    //       break;
    //     }
    //   }
    // }

    

    return key_frame_updated_;
  }

  // 用于判断关键帧
  const bool LidarHandler::IsKeyFrame(int64_t time) const
  {
    // 
    if (cloud_key_pos_->points.empty())
    {
      LOG(INFO) << "[first_lidar time_max] " << time * NS_TO_S;
      return true;
    }

    // 
    // 
    if (time >= trajectory_->GetActiveTime())
      return false;

    // 
    if ((time - cloud_key_pos_->back().timestamp) > keyframe_time_second_)
      return true;

    SE3d pose_cur = trajectory_->GetLidarPoseNURBS(time);
    SE3d pose_last = trajectory_->GetLidarPoseNURBS(cloud_key_pos_->back().timestamp);

    SE3d pose_cur_to_last = pose_last.inverse() * pose_cur;
    Eigen::AngleAxisd v(pose_cur_to_last.so3().unit_quaternion());

    double dist_meter = pose_cur_to_last.translation().norm();
    double angle_degree = v.angle() * (180. / M_PI);

    if (angle_degree > keyframe_angle_degree_ ||
        dist_meter > keyframe_dist_meter_)
    {
      //    LOG(INFO) << WHITE << "[ IsKeyFrame ] " << angle_degree << " : "
      //              << dist_meter;
      return true;
    }

    return false;
  }

  void LidarHandler::GetNearTimeKeyScanID(PosCloud::Ptr key_pos_selected,
                                           const int64_t cur_time) const
  {
    std::vector<double> t_added;
    t_added.reserve(key_pos_selected->size());
    for (size_t i = 0; i < key_pos_selected->size(); i++)
    {
      t_added.push_back(key_pos_selected->points[i].timestamp);
    }

    /// 
    int64_t map_start_time = cur_time - keyframe_search_time_;
    // float map_start_time = cloud_key_pos_->points.back().timestamp - keyframe_search_time_;
    for (int i = cloud_key_pos_->size() - 1; i >= 0; i--)
    {
      if (cloud_key_pos_->points[i].timestamp < map_start_time)
        break;
      //
      auto iter = std::find(t_added.begin(), t_added.end(),
                            cloud_key_pos_->points[i].timestamp);
      // 去掉和近邻搜索得到key_pos_selected相同的索引关键帧位置
      if (t_added.end() == iter)
      {
        key_pos_selected->push_back(cloud_key_pos_->points[i]);
      }
    }
  }

  void LidarHandler::GetNearDistKeyScanID(PosCloud::Ptr key_pos_selected)
  {
    PosCloud::Ptr nearby_key_pos(new PosCloud);

    std::vector<int> indices;
    std::vector<float> sqr_dists;
    pcl::KdTreeFLANN<PosPoint>::Ptr kdtree_nearby_key_pos(
        new pcl::KdTreeFLANN<PosPoint>());
    kdtree_nearby_key_pos->setInputCloud(cloud_key_pos_);
    kdtree_nearby_key_pos->radiusSearch(cloud_key_pos_->back(),
                                        (double)keyframe_search_radius_, indices,
                                        sqr_dists);
    for (const int &idx : indices)
    {
      nearby_key_pos->push_back(cloud_key_pos_->points[idx]);
    }
    if (cloud_key_pos_->size() > 20)
    {
      VoxelFilter<PosPoint> surrounding_key_pos_voxel_filter;
      surrounding_key_pos_voxel_filter.SetResolution(keyframe_density_map_);
      surrounding_key_pos_voxel_filter.SetInputCloud(nearby_key_pos);
      surrounding_key_pos_voxel_filter.Filter(key_pos_selected);
    }
    else
    {
      key_pos_selected = nearby_key_pos;
    }
  }

  void LidarHandler::TransformLiDARFeature(const LiDARFeature &lf_in,
                                            const Eigen::Matrix4d &T_in_to_out,
                                            LiDARFeature &lf_out) const
  {
    if (use_corner_feature_)
    {
      pcl::transformPointCloud(*lf_in.corner_features, *lf_out.corner_features,
                               T_in_to_out);
    }
    pcl::transformPointCloud(*lf_in.surface_features, *lf_out.surface_features,
                             T_in_to_out);

    if (update_full_cloud_)
    {
      pcl::transformPointCloud(*lf_in.full_cloud, *lf_out.full_cloud,
                               T_in_to_out);
    }

    lf_out.timestamp = lf_in.timestamp;
    lf_out.time_max = lf_in.time_max;
  }

  void LidarHandler::DownsampleLiDARFeature(const LiDARFeature &lf_in,
                                             LiDARFeature &lf_out) const
  {
    lf_out.Clear();
    lf_out.timestamp = lf_in.timestamp;
    lf_out.time_max = lf_in.time_max;

    if (use_corner_feature_)
    {
      VoxelFilter<PosPoint> corner_features_voxel_filter;
      corner_features_voxel_filter.SetResolution(corner_leaf_size_);
      corner_features_voxel_filter.SetInputCloud(lf_in.corner_features);
      corner_features_voxel_filter.Filter(lf_out.corner_features);
    }

    VoxelFilter<PosPoint> surface_features_voxel_filter;
    surface_features_voxel_filter.SetResolution(surface_leaf_size_);
    surface_features_voxel_filter.SetInputCloud(lf_in.surface_features);
    surface_features_voxel_filter.Filter(lf_out.surface_features);

    lf_out.full_cloud = lf_out.surface_features;
  }

  void LidarHandler::ExtractSurroundFeatures(const int64_t cur_time)
  {
    assert(!cloud_key_pos_->points.empty() && "no key scan to build local map");

    /// 经过距离阈值和时间阈值搜索得到的关键帧集合
    PosCloud::Ptr key_pos_selected(new PosCloud);
    GetNearDistKeyScanID(key_pos_selected);
    GetNearTimeKeyScanID(key_pos_selected, cur_time);
    LOG(INFO) << "[keysize] " << key_pos_selected->points.size();

    int64_t target_time = cloud_key_pos_->back().timestamp; // 
    feature_map_.Clear();
    feature_map_.timestamp = target_time;
    // first pos
    if (local_feature_container_.size() == 1)
    { // 
      int64_t kf_time = target_time;

      SE3d pose_cur_to_G = trajectory_->GetLidarPoseNURBS(kf_time); // TG_Lcur
      LiDARFeature lf_out;
      TransformLiDARFeature(local_feature_container_[kf_time],
                            pose_cur_to_G.matrix(), lf_out);

      if (use_corner_feature_)
      {
        *feature_map_.corner_features += *(lf_out.corner_features);
      }
      *feature_map_.surface_features += *(lf_out.surface_features);
      if (update_full_cloud_)
      {
        *feature_map_.full_cloud += *(lf_out.full_cloud);
      }
      
    }
    else
    {
      for (auto const &pos : key_pos_selected->points)
      {
        int64_t kf_time = pos.timestamp;

        // 在存储关键帧的关联容器中按照筛选过的帧索引查找进行build local map
        if (local_feature_container_.find(kf_time) !=
            local_feature_container_.end())
        {
          SE3d pose_cur_to_G = trajectory_->GetLidarPoseNURBS(kf_time); // TG_Lcur  //TODO：
          LiDARFeature lf_out;
          TransformLiDARFeature(local_feature_container_[kf_time],
                                pose_cur_to_G.matrix(), lf_out);

          if (use_corner_feature_)
          {
            *feature_map_.corner_features += *lf_out.corner_features;
          }

          *feature_map_.surface_features += *lf_out.surface_features;

          if (update_full_cloud_)
          {
            *feature_map_.full_cloud += *lf_out.full_cloud;
          }
          // 更新特征图的时间信息
          feature_map_.time_max =
              std::max(feature_map_.time_max, lf_out.time_max);
        }
      }
    }

    if (feature_map_.surface_features->size() < 10000)
    {
      feature_map_ds_.Clear();
      feature_map_ds_ = feature_map_;
    }
    else
    {
      DownsampleLiDARFeature(feature_map_, feature_map_ds_);
    }

    LOG(INFO) << "[feature_map_ds] "
              << "surface cloud map : "
              << feature_map_ds_.surface_features->size()
              << "; cornor cloud map: " << feature_map_ds_.corner_features->size()
              << "; feature map time : [" << feature_map_ds_.timestamp << ", "
              << feature_map_ds_.time_max << "].";
    
    double t0 = omp_get_wtime();
    SetTargetMap(feature_map_ds_);
    double t1 = omp_get_wtime();
    std::cout << "Build kdtree : " << (t1 - t0) * 1000 << " ms" << std::endl;
  }
// use ikd-tree
  void LidarHandler::SetTargetMap(const LiDARFeature &feature_map)
  {
    assert(!feature_map.surface_features->empty() &&
           "[SetTargetMap] input surface_features are empty");
    assert(!(use_corner_feature_ && feature_map.corner_features->empty()) &&
           "[SetTargetMap] input corner_features are empty");

    if (use_corner_feature_)
      kdtree_corner_map_->setInputCloud(feature_map.corner_features);

    kdtree_surface_map_->setInputCloud(feature_map.surface_features);
  }

  // use ikd-tree to near-search
  bool LidarHandler::FindCorrespondence(const LiDARFeature &lf_cur,
                                         const LiDARFeature &lf_cur_in_M)
  {
    point_correspondence_.clear();
    point_correspondence_.reserve(lf_cur.surface_features->size()); // 

    //
    if ((use_corner_feature_ &&
         lf_cur.corner_features->size() < size_t(edge_min_valid_num_)) &&
        lf_cur.surface_features->size() < size_t(surf_min_valid_num_))
    {
      LOG(WARNING) << "[FindCorrespondence] No enough feature points ! Corner: "
                   << lf_cur.corner_features->size()
                   << "; Surface: " << lf_cur.surface_features->size();
      return false;
    }

    //  
    size_t corner_step = lf_cur.corner_features->size() / 1500 + 1;
    LOG(INFO) << "[corner_step] " << corner_step;
//     if (use_corner_feature_ && feature_map_ds_.corner_features->size() > 10)
//     {
//       for (size_t i = 0; i < lf_cur.corner_features->size(); i += corner_step)
//       {
//         PosPoint point_inM = lf_cur_in_M.corner_features->points[i];
//         std::vector<int> k_indices;
//         std::vector<float> k_sqr_dists;
//         kdtree_corner_map_->nearestKSearch(point_inM, 5, k_indices, k_sqr_dists);

//         if (k_sqr_dists[4] > 1.0)
//           continue;

//         cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
//         cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
//         cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            // 计算五个近邻点的质心
//         Eigen::Vector3d cp(0, 0, 0);
//         for (int j = 0; j < 5; j++)
//         {
//           cp[0] += feature_map_ds_.corner_features->points[k_indices[j]].x;
//           cp[1] += feature_map_ds_.corner_features->points[k_indices[j]].y;
//           cp[2] += feature_map_ds_.corner_features->points[k_indices[j]].z;
//         }
//         cp /= 5.0;
            // 计算这五个点相对于质心‘cp’的协方差元素
//         float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
//         for (int j = 0; j < 5; j++)
//         {
//           float ax =
//               feature_map_ds_.corner_features->points[k_indices[j]].x - cp[0];
//           float ay =
//               feature_map_ds_.corner_features->points[k_indices[j]].y - cp[1];
//           float az =
//               feature_map_ds_.corner_features->points[k_indices[j]].z - cp[2];

//           a11 += ax * ax;
//           a12 += ax * ay;
//           a13 += ax * az;
//           a22 += ay * ay;
//           a23 += ay * az;
//           a33 += az * az;
//         }
//         a11 /= 5;
//         a12 /= 5;
//         a13 /= 5;
//         a22 /= 5;
//         a23 /= 5;
//         a33 /= 5;

//         matA1.at<float>(0, 0) = a11;
//         matA1.at<float>(0, 1) = a12;
//         matA1.at<float>(0, 2) = a13;
//         matA1.at<float>(1, 0) = a12;
//         matA1.at<float>(1, 1) = a22;
//         matA1.at<float>(1, 2) = a23;
//         matA1.at<float>(2, 0) = a13;
//         matA1.at<float>(2, 1) = a23;
//         matA1.at<float>(2, 2) = a33;

//         /// 计算matA1的特征值 matD1
//         cv::eigen(matA1, matD1, matV1);

//         /// 判断最大的特征值是否大于第二大特征值的3倍。如果是，表示这些点在某个方向上具有明显的线性结构（即角点特征）
//         if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
//         {
// #if 1
//           // 
//           float x0 = point_inM.x;
//           float y0 = point_inM.y;
//           float z0 = point_inM.z;
//           // 线段的两个顶点坐标
//           float x1 = cp[0] + 0.1 * matV1.at<float>(0, 0);
//           float y1 = cp[1] + 0.1 * matV1.at<float>(0, 1);
//           float z1 = cp[2] + 0.1 * matV1.at<float>(0, 2);
//           float x2 = cp[0] - 0.1 * matV1.at<float>(0, 0);
//           float y2 = cp[1] - 0.1 * matV1.at<float>(0, 1);
//           float z2 = cp[2] - 0.1 * matV1.at<float>(0, 2);

//           // 点到(x1,y1,z1)的向量和线段向量叉积的模长
//           float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

//           // 线段的长度
//           float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

//           //
//           float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

//           float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

//           float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

//           // 
//           float ld2 = a012 / l12;

//           // 
//           float s = 1 - 0.9 * fabs(ld2); 

//           if (s <= 0.1)
//           {
//             std::cout << YELLOW << "bad corner association\n"
//                       << RESET;
//             continue;
//           }
// #endif

//           Eigen::Vector3d normal(matV1.at<float>(0, 0), matV1.at<float>(0, 1),
//                                  matV1.at<float>(0, 2));
//           // double plane_d = -normal.dot(cp);

//           PointCorrespondence point_cor;
//           point_cor.geo_type = Line;
//           point_cor.t_point = lf_cur.corner_features->points[i].timestamp;
//           point_cor.t_map = feature_map_.timestamp;
//           point_cor.geo_point = cp;
//           point_cor.geo_normal = normal;
//           point_cor.point = Eigen::Vector3d(lf_cur.corner_features->points[i].x,
//                                             lf_cur.corner_features->points[i].y,
//                                             lf_cur.corner_features->points[i].z);
//           point_cor.scale = s;
//           point_correspondence_.push_back(point_cor);
//         }
//       }
//     }
    int corner_feature_num = point_correspondence_.size();

    /// lio-sam
    map_corrs_viewer.clear();
    size_t surf_step = lf_cur.surface_features->size() / 1500 + 1;
    LOG(INFO) << "[surf_step] " << surf_step;
    for (size_t i = 0; i < lf_cur.surface_features->size(); i += surf_step)
    {
      if (lf_cur.surface_features->points[i].timestamp < 0)
        continue;
      
      // 查找每个面点在特征图中对应的周围五个点
      PosPoint point_inM = lf_cur_in_M.surface_features->points[i];
      
      std::vector<int> k_indices;
      std::vector<float> k_sqr_dists(NUM_MATCH_POINTS);
      // std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
      KD_TREE<PosPoint>::PointVector &points_near = Nearest_Points[i];
      if(first_initial_ikdtree)
      {
        kdtree_surface_map_->nearestKSearch(point_inM, 5, k_indices, k_sqr_dists);
      }
      else
      {
        // std::cout << "Debug \n";
        ikdtree.Nearest_Search(point_inM, NUM_MATCH_POINTS, points_near, k_sqr_dists);
        // std::cout << points_near.size() << std::endl; 
      }


      // 距离阈值判断是否为有效点
      if (k_sqr_dists[4] > 1.0)
        continue;

      Eigen::Matrix<float, 5, 3> matA0;
      Eigen::Matrix<float, 5, 1> matB0;
      Eigen::Vector3f matX0;
      matA0.setZero();
      matB0.fill(-1);
      matX0.setZero();

      if (points_near.size() < 5)
        continue;

      for (int j = 0; j < 5; j++)
      {
        if(first_initial_ikdtree)
        {
          matA0(j, 0) = feature_map_ds_.surface_features->points[k_indices[j]].x;
          matA0(j, 1) = feature_map_ds_.surface_features->points[k_indices[j]].y;
          matA0(j, 2) = feature_map_ds_.surface_features->points[k_indices[j]].z;
        }else{
          matA0(j, 0) = points_near[j].x;
          matA0(j, 1) = points_near[j].y;
          matA0(j, 2) = points_near[j].z;
        }
      }

      matX0 = matA0.colPivHouseholderQr().solve(matB0);

      /// 
      float pa = matX0(0, 0);
      float pb = matX0(1, 0);
      float pc = matX0(2, 0);
      float pd = 1;

      /// 
      float ps = sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      /// 
      for (int j = 0; j < 5; j++)
      { 
        if (first_initial_ikdtree)
        {
          if (fabs(pa * feature_map_ds_.surface_features->points[k_indices[j]].x +
                 pb * feature_map_ds_.surface_features->points[k_indices[j]].y +
                 pc * feature_map_ds_.surface_features->points[k_indices[j]].z +
                 pd) > 0.2)
          {
            planeValid = false;
            break;
          }

        }
        else
        {
          if (fabs(pa * points_near[j].x +
                  pb * points_near[j].y +
                  pc * points_near[j].z +
                  pd) > 0.2)
          {
            planeValid = false;
            break;
          }
        }
      }

      // 
      if (planeValid)
      {
        float pd2 = pa * point_inM.x + pb * point_inM.y + pc * point_inM.z + pd;

        float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(point_inM.x * point_inM.x + point_inM.y * point_inM.y + point_inM.z * point_inM.z));

        if (s <= 0.1)
        {
          std::cout << YELLOW << "bad surface association\n"
                    << RESET;
          continue;
        }

        PointCorrespondence point_cor;
        point_cor.geo_type = Plane;
        point_cor.t_point = lf_cur.surface_features->points[i].timestamp;
        point_cor.t_map = lf_cur.timestamp; // feature_map_ds_
        point_cor.point = Eigen::Vector3d(lf_cur.surface_features->points[i].x,
                                          lf_cur.surface_features->points[i].y,
                                          lf_cur.surface_features->points[i].z);
        point_cor.geo_plane = Eigen::Vector4d(pa, pb, pc, pd);
        point_cor.scale = s;
        point_correspondence_.push_back(point_cor);

        double color =
            point_cor.point[0] + point_cor.point[1] + point_cor.point[2];

        for (int i = 0; i < 5; ++i)
        {
          VPoint vp;
          vp.x = matA0(i, 0);
          vp.y = matA0(i, 1);
          vp.z = matA0(i, 2);
          vp.intensity = color;
          map_corrs_viewer.push_back(vp);
        }
      }
    }

    int surf_feature_num = point_correspondence_.size() - corner_feature_num;
    LOG(INFO) << "point correspondence: " << point_correspondence_.size()
              << "; corner/surfel " << corner_feature_num << "/"
              << surf_feature_num;
    return true;
  }

  // TODO 
  void LidarHandler::DownSampleCorrespondence()
  {
    int step = cor_downsample_;
    // if (point_correspondence_.size() < 1200)
    // step = 1;

    size_t reserve_size = point_correspondence_.size() / step + 1;
    point_correspondence_ds_.clear();
    point_correspondence_ds_.reserve(reserve_size);
    for (size_t i = 0; i < point_correspondence_.size(); i += step)
    {
      point_correspondence_ds_.push_back(point_correspondence_.at(i));
    }
  }

  // void LidarHandler::FindNearbyKeyFrames(const int key, const int search_num,
  //                                         PosCloud::Ptr &cloud_in_G) const
  // {
  //   cloud_in_G->clear();

  //   int keyscan_size = cloud_key_pos_xy_->size();
  //   for (int i = -search_num; i <= search_num; i++)
  //   {
  //     int key_index = key + i;
  //     if (key_index < 0 || key_index >= keyscan_size)
  //       continue;

  //     int64_t kf_time = cloud_key_pos_xy_->points.at(key_index).timestamp;
  //     if (local_feature_container_all_ds_.find(kf_time) ==
  //         local_feature_container_all_ds_.end())
  //       continue;

  //     const auto &local_lf = local_feature_container_all_ds_.at(kf_time);

  //     PosCloud local_cloud, global_cloud;
  //     if (!local_lf.full_cloud->empty())
  //     {
  //       local_cloud += *local_lf.full_cloud;
  //     }
  //     else
  //     {
  //       local_cloud += *local_lf.surface_features;
  //     }

  //     Eigen::Matrix4d T_cur_to_G = trajectory_->GetLidarPoseNURBS(kf_time).matrix();
  //     pcl::transformPointCloud(local_cloud, global_cloud, T_cur_to_G);
  //     *cloud_in_G += global_cloud;
  //   }
  // }

  // void LidarHandler::UpdateCloudKeyPos(
  //     const std::pair<int, int> &cur_wrt_history)
  // {
  //   // update key_pose
  //   int64_t t_cur = cloud_key_pos_xy_->points[cur_wrt_history.first].timestamp;
  //   int64_t t_history =
  //       cloud_key_pos_xy_->points[cur_wrt_history.second].timestamp;

  //   for (size_t i = 0; i < cloud_key_pos_xy_->size(); i++)
  //   {
  //     int64_t kf_time = cloud_key_pos_xy_->points[i].timestamp;
  //     if (kf_time < t_history || kf_time > t_cur)
  //     {
  //       continue;
  //     }

  //     Eigen::Vector3d p = trajectory_->GetLidarPoseNURBS(kf_time).translation();
  //     cloud_key_pos_->points[i].x = p(0);
  //     cloud_key_pos_->points[i].y = p(1);
  //     cloud_key_pos_->points[i].z = p(2);
  //     cloud_key_pos_xy_->points[i].x = p(0);
  //     cloud_key_pos_xy_->points[i].y = p(1);
  //   }

  //   // 
  //   key_frame_updated_ = true;
  // }

} // namespace cocolic
