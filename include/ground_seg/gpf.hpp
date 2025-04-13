#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Dense>

class GroundSegmentationNode {
public:
    GroundSegmentationNode(ros::NodeHandle &nh) {

        // ROS接口
        sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            "/lio/source_dense_cloud", 
            1, 
            &GroundSegmentationNode::cloudCallback, 
            this
        );

        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
        non_ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/non_ground_cloud", 1);

        ground_plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        ground_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        noground_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

        ROS_INFO("Open segground\n");
        // ROS_INFO_STREAM(ros::this_node::getName() << " initialized with:\n");
            // << "  distance_threshold: " << distance_threshold_ << "\n"
            // << "  max_iterations: " << max_iterations_);
    }
    ~GroundSegmentationNode(){
        saveGroundmap();

    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg) {
        // 1. 输入检查
        if (!input_msg || input_msg->data.empty()) {
            ROS_WARN_THROTTLE(5.0, "Empty input point cloud received");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
        // 2. 转换点云格式
        try {
            pcl::fromROSMsg(*input_msg, *cloud);
            if (cloud->empty()) {
                ROS_WARN_THROTTLE(5.0, "Converted empty point cloud");
                return;
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Point cloud conversion failed: " << e.what());
            return;
        }
        
        extract_initial_seeds(*cloud);
        // 迭代
        for(int i = 0; i < N_iter; i++){
            if(i == 0) th_dist_ = 0.2;
            if(i == 1) th_dist_ = 0.1;
            if(i == 2) th_dist_ = 0.1;
            estimate_plane_parameter();
            ground_plane_cloud->clear();
            ground_point_cloud->clear();
            noground_point_cloud->clear();
            extract_plane_cloud(*cloud,ground_point_cloud,noground_point_cloud);
               
        }

        ground_vec.push_back(*ground_point_cloud);
        noground_vec.push_back(*noground_point_cloud);
        // 6. 发布结果
        publishResults(input_msg->header, ground_point_cloud, noground_point_cloud);

        ground_plane_cloud->clear();
        ground_point_cloud->clear();
        noground_point_cloud->clear();

    }

    void extract_initial_seeds(const pcl::PointCloud<pcl::PointXYZ> &input_point_cloud){
    double height_sum = 0;
    int cnt = 0;
    for (int i = 0; i < input_point_cloud.points.size(); i++){
        height_sum += input_point_cloud.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? height_sum / cnt : 0;	// 求解其高度均值					
    for (int i = 0; i < input_point_cloud.points.size(); i++){
        if (input_point_cloud.points[i].z < lpr_height + 0.6){ //0.6 -> raw lidar's z
            ground_plane_cloud->points.push_back(input_point_cloud.points[i]);
        }
    }
}

void estimate_plane_parameter(void){
    
    if (!ground_plane_cloud || ground_plane_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Empty ground plane cloud received");
        plane_normal_.setZero();
        dis_ = 0.0;
        th_dist_d_ = 0.0;
        return;
    }

    // 2. 计算均值和协方差矩阵
    Eigen::Matrix3f cov_matrix;
    Eigen::Vector4f pc_mean;
    try {
        pcl::computeMeanAndCovarianceMatrix(*ground_plane_cloud, cov_matrix, pc_mean);
        
        // 3. SVD分解（仅计算U矩阵）
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(
            cov_matrix, 
            Eigen::ComputeFullU | Eigen::ComputeThinV
        );
        
        // 4. 获取最小特征值对应的特征向量（平面法向量）
        plane_normal_ = svd.matrixU().col(2);
        
        // 5. 确保法向量朝上（与传感器坐标系一致）
        if (plane_normal_(2) < 0) {
            plane_normal_ = -plane_normal_;
        }
        
        // 6. 计算平面距离参数 ax + by + cz + d = 0
        const Eigen::Vector3f seeds_mean = pc_mean.head<3>();
        dis_ = -(plane_normal_.transpose() * seeds_mean)(0, 0);
        
        // 7. 更新距离阈值
        th_dist_d_ = th_dist_ - dis_;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Plane estimation failed: " << e.what());
        plane_normal_.setZero();
        dis_ = 0.0;
        th_dist_d_ = 0.0;
        
    }
    
    return;
}

void extract_plane_cloud(const pcl::PointCloud<pcl::PointXYZ> input_point_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr & ground_point_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr & noground_point_cloud){
    //point cloud to matrix
    Eigen::MatrixXf points_matrix(input_point_cloud.points.size(), 3);
    int j = 0;
    for (auto p : input_point_cloud.points){
        points_matrix.row(j++) << p.x, p.y, p.z;
    }
    Eigen::VectorXf result_dis = points_matrix * plane_normal_;
    for (int r = 0; r < result_dis.rows(); r++){
        if (result_dis[r] < th_dist_d_){
            ground_point_cloud->points.push_back(input_point_cloud[r]);
        }
        else{
            noground_point_cloud->points.push_back(input_point_cloud[r]);
        }
    }
    *ground_plane_cloud = *ground_point_cloud;
    return;
}


    void publishResults(const std_msgs::Header& header,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& non_ground) {
        if (!ground->empty()) {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*ground, msg);
            msg.header = header;
            ground_pub_.publish(msg);
        }

        if (!non_ground->empty()) {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*non_ground, msg);
            msg.header = header;
            non_ground_pub_.publish(msg);
        }
    }
public:
    void saveGroundmap()
    {
       
      std::cout << "Saving Ground map ....." << std::endl;
      pcl::PointCloud<pcl::PointXYZ> groundcloud;
      pcl::PointCloud<pcl::PointXYZ> nogroundcloud;
      int frames = 0, pt_cnt; 
      for (int i = 0; i <  ground_vec.size(); i++)
      {
        if (i % 1 ==0){
          groundcloud += ground_vec[i];
          frames++;
          pt_cnt += groundcloud.size();
        }
      }
      for(auto &cloud : noground_vec)
            nogroundcloud += cloud;
      std::cout << "Frame's num:" << frames << " , All points"<<" : " << pt_cnt << std::endl;;
     
      pcl::io::savePCDFileBinary("/home/h/hong.MD.degree/lico_ws/src/LICO-mid360/PCD/ground.pcd", groundcloud);
      pcl::io::savePCDFileBinary("/home/h/hong.MD.degree/lico_ws/src/LICO-mid360/PCD/noground.pcd", nogroundcloud);
      std::cout << "Finish save ground map. \n";
      return;
    
  
    }


    // 成员变量
    ros::Subscriber sub_;
    ros::Publisher ground_pub_, non_ground_pub_;
    pcl::SACSegmentation<pcl::PointXYZ> seg_;
    std::mutex seg_mutex_;
    double distance_threshold_ = 0.05;
    int max_iterations_ = 1000;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_cloud;
     pcl::PointCloud<pcl::PointXYZ>::Ptr ground_point_cloud;
      pcl::PointCloud<pcl::PointXYZ>::Ptr noground_point_cloud;

      std::vector<pcl::PointCloud<pcl::PointXYZ>> ground_vec;
      std::vector<pcl::PointCloud<pcl::PointXYZ>> noground_vec;

      Eigen::Vector3f plane_normal_;
      float th_dist_d_;
      float th_dist_ = 0.1;
      float dis_ = 0.;
      int N_iter = 2;

};