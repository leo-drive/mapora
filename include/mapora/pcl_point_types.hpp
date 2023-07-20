// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__PCL_Pointtypes
#define MAPORA__PCL_Pointtypes

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace mapora
{
namespace pcl_point_types{
using PointPcl = pcl::PointXYZI;
using CloudPcl = pcl::PointCloud<PointPcl>;
}  // namespace pcl_point_types
}  // namespace mapora

#endif  // MAPORA__PCL_Pointtypes
