#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
 
ros::Publisher sync_cloud_pub;
ros::Publisher sync_imu_pub;

void livoxCallback(const livox_ros_driver2::CustomMsgConstPtr& cloud_msg)
{
    if(cloud_msg->point_num == 0)
        return;
    sync_cloud_pub.publish(cloud_msg);
   
}
void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    
    sync_imu_pub.publish(imu_msg);
}

void callback(const livox_ros_driver2::CustomMsgConstPtr& cloud_msg,const sensor_msgs::ImuConstPtr& imu_msg)
{
    sync_cloud_pub.publish(cloud_msg);
    sync_imu_pub.publish(imu_msg);
}
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "livox_imu_sync_node");
    ros::NodeHandle nh;
    std::cout << "====== livox repub node ======\n"; 
    sync_cloud_pub = nh.advertise<livox_ros_driver2::CustomMsg>("/sync/livox/lidar", 1000);
    sync_imu_pub = nh.advertise<sensor_msgs::Imu>("/sync/livox/imu", 1000);
    ros::Subscriber livox_sub = nh.subscribe("/livox/lidar", 2000, &livoxCallback);
    ros::Subscriber imu_sub = nh.subscribe("/livox/imu", 2000, &imuCallback);
    // 创建tb订阅器，订阅Livox点云和IMU数据
    // message_filters::Subscriber<livox_ros_driver2::CustomMsg> cloud_sub(nh, "/livox/lidar", 1000);
    // message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/livox/imu", 20000);

    // typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::CustomMsg, sensor_msgs::Imu> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), cloud_sub, imu_sub);
    // sync.setMaxIntervalDuration(ros::Duration(0.01));
    // sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();

    return 0;
}