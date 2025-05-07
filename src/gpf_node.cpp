/*
 * @Author: piluohong 1912694135@qq.com
 * @Date: 2025-05-06 19:38:36
 * @LastEditors: piluohong 1912694135@qq.com
 * @LastEditTime: 2025-05-06 19:39:53
 * @FilePath: /hong.MD.degree/lico_ws/src/LICO-mid360/src/gpf_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <csignal>
#include <ros/package.h>
#include <ros/ros.h>
#include <ground_seg/gpf.hpp>

int main(int argc, char **argv){

    ros::init(argc, argv, "gpf_node");
  ros::NodeHandle nh("~");

   GroundSegmentationNode segnode(nh);
   ros::spin();
   return 0;

}