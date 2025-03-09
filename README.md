# LICO-mid360 using for mid360, it's derived from Coco-LIC.

* Now there has some imrpoved things:
* Added rosbag function;
* Added ikd-tree and ivox to manage local-map.
* Some bug we found:
* When open VIO, the accuracy of the fused odometry is declined.
* Todo:
* I will improved the VIO part.

## Prerequisites

+ ROS（tested with noetic）
+ Eigen 3.3.7
+ Ceres 2.0.0
+ OpenCV 4
+ PCL >= 1.13
+ [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)
+ yaml-cpp

## Install

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/catkin_ws && catkin_make
cd ~/catkin_ws/src
git clone https://github.com/piluohong/LICO-mid360.git
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/Coco-LIC && mkdir data
```

## Noted

+ pcl >= 1.13 has some bugs with pcl-1.10, we suggested build pcl>=1.13 separtely.

## Run

+ Download [R3LIVE dataset](https://github.com/ziv-lin/r3live_dataset) or [FAST-LIVO dataset](https://connecthkuhk-my.sharepoint.com/personal/zhengcr_connect_hku_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fzhengcr%5Fconnect%5Fhku%5Fhk%2FDocuments%2FFAST%2DLIVO%2DDatasets&ga=1) or [NTU-VIRAL dataset](https://ntu-aris.github.io/ntu_viral_dataset/) or [LVI-SAM dataset](https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV).
+ Configure parameters in the `config/ct_odometry_xxx.yaml` file.

  - `log_path`: the path to log
  - `config_path`: the path of `config` folder
  - `bag_path`: the file path of rosbag
+ Run on R3LIVE dataset for example.

  ```shell
  roslaunch cocolic odometry.launch config_path:=config/ct_odometry_r3live.yaml
  ```

  The estimated trajectory is saved in the folder `./src/Coco-LIC/data`.

## Acknowledgement

Thanks for Coco-LIC, LIO-SAM, FAST-LIO2 && Faster-LIO
