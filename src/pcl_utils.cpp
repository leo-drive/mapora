// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include <string>
#include <memory>
#include <algorithm>

#include <mapora/pcl_utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

namespace mapora
{
using PointPcl = PclUtils::PointPcl;
using CloudPcl = PclUtils::CloudPcl;

using Point = PclUtils::Point;
using PointCloud2 = PclUtils::PointCloud2;

// PCL to ROS2

PclUtils::Point PclUtils::point_pcl_to_point(const PointPcl & point_pcl)
{
  return Point{point_pcl.x, point_pcl.y, point_pcl.z, point_pcl.intensity};
}

PointCloud2::SharedPtr PclUtils::cloud_pcl_to_pointcloud2(
  const CloudPcl::ConstPtr & cloud_in, const std::string & frame_id)
{
  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<Point>;
  PointCloud2::SharedPtr cloud_ptr = std::make_shared<PointCloud2>();
  CloudModifier cloud_modifier_trans(*cloud_ptr, frame_id);
  cloud_modifier_trans.resize(cloud_in->size());
  std::transform(
    cloud_in->points.cbegin(),
    cloud_in->points.cend(),
    cloud_modifier_trans.begin(),
    point_pcl_to_point);
  return cloud_ptr;
}

// ROS2 to PCL

PointPcl PclUtils::point_to_point_pcl(const Point & point_in)
{
  PointPcl point_pcl;
  point_pcl.x = point_in.x;
  point_pcl.y = point_in.y;
  point_pcl.z = point_in.z;
  point_pcl.intensity = point_in.intensity;
  return point_pcl;
}

CloudPcl::Ptr PclUtils::pointcloud2_to_cloud_pcl(const PointCloud2::ConstSharedPtr & cloud_in)
{
  using CloudView = point_cloud_msg_wrapper::PointCloud2View<Point>;
  CloudView cloud_view(*cloud_in);
//  std::cout << "cloud_view: " << cloud_view.size() << std::endl;
  CloudPcl::Ptr cloud(new CloudPcl);
  cloud->points.resize(cloud_view.size());
  std::transform(cloud_view.cbegin(), cloud_view.cend(), cloud->points.begin(), point_to_point_pcl);
  return cloud;
}


// Vector Points to PCL

CloudPcl::Ptr PclUtils::points_to_cloud_pcl(const std::vector<point_types::PointXYZI> & points_in)
{
  CloudPcl::Ptr cloud(new CloudPcl);
  cloud->resize(points_in.size());
  std::transform(
    points_in.cbegin(), points_in.cend(), cloud->begin(), PclUtils::point_to_point_pcl);
  return cloud;
}

CloudPcl::Ptr PclUtils::downsample_approx(const CloudPcl::ConstPtr & cloud_in, float leaf_size)
{
  CloudPcl::Ptr cloud_out(new CloudPcl);
  pcl::ApproximateVoxelGrid<PointPcl> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  approximate_voxel_filter.setInputCloud(cloud_in);
  approximate_voxel_filter.filter(*cloud_out);
  return cloud_out;
}

CloudPcl::Ptr PclUtils::downsample(const CloudPcl::ConstPtr & cloud_in, float leaf_size)
{
  CloudPcl::Ptr cloud_out(new CloudPcl);
  pcl::VoxelGrid<PointPcl> voxel_filter;
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.setInputCloud(cloud_in);
  voxel_filter.filter(*cloud_out);
  return cloud_out;
}


}  // namespace mapora
