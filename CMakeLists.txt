cmake_minimum_required(VERSION 3.0.2)
project(cocolic)

add_definitions(-w)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Wno-sign-compare -Wno-unused -Wno-comment -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  eigen_conversions
  cv_bridge
  roslib
  rosbag
  tf
  message_generation
  image_transport
  livox_ros_driver2
)

# FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization thread)
if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

include_directories("/home/hyq/slam/3rdku/pcl-1.14.0-installpath/include/pcl-1.14") #pcl-1.13 与 自带的pcl-1.10冲突，使用该指令
set(PCL_DIR "/home/hyq/slam/3rdku/pcl-1.14.0-installpath/share/pcl-1.14")
set(Ceres_DIR "/home/hyq/slam/3rdku/ceres-2.0.0-installpath/lib/cmake")
# set(ncnn_DIR "/home/hyq/slam/3rdku/ncnn/build/install/lib/cmake/ncnn" CACHE PATH "Directory that contains ncnnConfig.cmake")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
# find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(ncnn REQUIRED)
# find_package(PCL 1.13.0 REQUIRED HINTS ${PCL_DIR})

# LOOP
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

# find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  feature_cloud.msg
  imu_array.msg
  pose_array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS std_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  # ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR} #loop
  # /usr/local/include/pcl-1.13
  src/camera
  src/camera/loam/include
  src/camera/tools/
  src/camera/rgb_map
)

link_directories(
  # ${PCL_LIBRARY_DIRS}
"/home/hyq/slam/3rdku/pcl-1.14.0-installpath/lib"
)

list(APPEND thirdparty_libraries
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        # ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SERIALIZATION_LIBRARY} # serialization
        # ${}
)

add_library(spline_lib src/spline/trajectory.cpp)
target_link_libraries(spline_lib ${thirdparty_libraries})

# add_library(ikdtree_lib src/camera/loam/include/ikd-Tree/ikd_Tree.cpp)
# target_link_libraries(ikdtree_lib ${thirdparty_libraries})

file(GLOB lidar_odometry_files
    "src/lidar/lidar_handler.cpp"
    "src/lidar/velodyne_feature_extraction.cpp"
    "src/lidar/livox_feature_extraction.cpp"  
    "include/ikd-tree/ikd_Tree.cpp"
    "include/ivox3d/ivox3d_node.hpp"
)
add_library(lidar_lib ${lidar_odometry_files})
target_link_libraries(lidar_lib spline_lib ${thirdparty_libraries})

file(GLOB r3live_files
    "src/camera/loam/*.cpp"
    "src/camera/optical_flow/*.cpp"
    "src/camera/rgb_map/*.cpp"
    "src/camera/*.cpp"
    # "src/ncnn_node/ncnn_image.hpp"
    "include/tracking.cpp"
)
add_library(r3live_lib ${r3live_files})
target_link_libraries(r3live_lib ${thirdparty_libraries})

add_executable(odometry_node 
   src/odometry_node.cpp
   src/imu/imu_state_estimator.cpp
   src/imu/imu_initializer.cpp
   src/odom/msg_manager.cpp
   src/odom/trajectory_manager.cpp
   src/odom/trajectory_estimator.cpp
   src/odom/odometry_manager.cpp
   src/odom/factor/analytic_diff/marginalization_factor.cpp
   src/utils/parameter_struct.cpp
)
target_link_libraries(odometry_node 
   spline_lib 
   lidar_lib 
   r3live_lib
   ${thirdparty_libraries}
  #  fmt::fmt
  "/home/hyq/slam/3rdku/pcl-1.14.0-installpath/lib"
)

add_executable(alaserPGO 
  src/PGO/laserPosegraphOpt.cpp
  include/scancontext/Scancontext.cpp
)
target_link_libraries(alaserPGO 
  ${catkin_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  lidar_lib
  pcl_registration
  "/home/hyq/slam/3rdku/pcl-1.14.0-installpath/lib"
  gtsam
)

#Ncnn

option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" ON)

if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()