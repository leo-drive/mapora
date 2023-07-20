// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__PCL_UTILS_HPP_
#define MAPORA__PCL_UTILS_HPP_

#include <string>
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <mapora/point_xyzi.hpp>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

namespace mapora
{
class PclUtils
{
public:
  using PointPcl = pcl::PointXYZI;
  using CloudPcl = pcl::PointCloud<PointPcl>;

  using Point = point_types::PointXYZI;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  // PCL to ROS2
  static Point point_pcl_to_point(const PointPcl & point_pcl);
  static PointCloud2::SharedPtr cloud_pcl_to_pointcloud2(
    const CloudPcl::ConstPtr & cloud_in, const std::string & frame_id);

  // ROS2 to PCL
  static PointPcl point_to_point_pcl(const Point & point_in);
  static CloudPcl::Ptr pointcloud2_to_cloud_pcl(const PointCloud2::ConstSharedPtr & cloud_in);

  // Vector Points to PCL
  static CloudPcl::Ptr points_to_cloud_pcl(const std::vector<Point> & points_in);

  // Downsample
  static CloudPcl::Ptr downsample_approx(const CloudPcl::ConstPtr & cloud_in, float leaf_size);
  static CloudPcl::Ptr downsample(const CloudPcl::ConstPtr & cloud_in, float leaf_size);


  template <typename PointT = Point>
  static double distance(const PointT & first, const PointT & second)
  {
    return std::hypot(first.x - second.x, first.y - second.y, first.z - second.z);
  }

  template <typename PointT = Point> static double norm(const PointT & point)
  {
    return std::hypot(point.x, point.y, point.z);
  }
};

}  // namespace mapora

#endif  // MAPORA__PCL_UTILS_HPP_
