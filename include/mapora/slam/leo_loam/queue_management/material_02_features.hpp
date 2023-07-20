// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_02_FEATURES_HPP_
#define MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_02_FEATURES_HPP_

#include <std_msgs/msg/header.hpp>

#include <memory>

#include "mapora/slam/leo_loam/producer_consumer_master.hpp"
#include "mapora/pcl_point_types.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class MaterialFeatures : public ProConMaterialBase
{
public:
  using Ptr = std::shared_ptr<MaterialFeatures>;
  using Point = pcl_point_types::PointPcl;
  using Cloud = pcl_point_types::CloudPcl;

  Cloud::ConstPtr cloud_full;
  Cloud::ConstPtr cloud_edge;
  Cloud::ConstPtr cloud_edge_pot;
  Cloud::ConstPtr cloud_planar;
  Cloud::ConstPtr cloud_planar_pot;
  std_msgs::msg::Header::ConstSharedPtr header_in;

  double Roll;
  double Pitch;
  double Yaw;

  void Log() override
  {
    std::cout << "count: " << count << std::endl;
    std::cout << "cloud_full: " << cloud_full->points.size() << std::endl;
    std::cout << "cloud_edge: " << cloud_edge->points.size() << std::endl;
    std::cout << "cloud_edge_pot: " << cloud_edge_pot->points.size() << std::endl;
    std::cout << "cloud_planar: " << cloud_planar->points.size() << std::endl;
    std::cout << "cloud_planar_pot: " << cloud_planar_pot->points.size() << std::endl;
  }
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora

#endif  // MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_02_FEATURES_HPP_
