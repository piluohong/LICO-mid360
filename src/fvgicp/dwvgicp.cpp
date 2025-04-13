/*
 * @Author: piluohong 1912694135@qq.com
 * @Date: 2025-04-13 14:45:14
 * @LastEditors: piluohong 1912694135@qq.com
 * @LastEditTime: 2025-04-13 15:20:32
 * @FilePath: /hong.MD.degree/lico_ws/src/LICO-mid360/src/fvgicp/dwvgicp.cpp
//  * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "fvgicp/dwvgicp.hpp"
#include "fvgicp/dwvgicp_impl.hpp"
#include <utils/mypcl_cloud_type.h>

template class fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::FastVGICP<pcl::PointNormal, pcl::PointNormal>;
// template class fast_gicp::FastVGICP<VPoint,VPoint>;

