/*
 * @Author: piluohong 1912694135@qq.com
 * @Date: 2025-04-13 14:45:14
 * @LastEditors: piluohong 1912694135@qq.com
 * @LastEditTime: 2025-04-13 15:20:28
 * @FilePath: /hong.MD.degree/lico_ws/src/LICO-mid360/src/fvgicp/lsq_registration.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "fvgicp/lsq_registration.hpp"
#include "fvgicp/lsq_registration_impl.hpp"
#include <utils/mypcl_cloud_type.h>

template class fast_gicp::LsqRegistration<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::LsqRegistration<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::LsqRegistration<pcl::PointNormal, pcl::PointNormal>;
// template class fast_gicp::LsqRegistration<VPoint,VPoint>;
