// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_03_INIT_TRANSFORMATIONS_HPP_
#define MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_03_INIT_TRANSFORMATIONS_HPP_

#include <Eigen/Core>

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
class MaterialInitialTransformations : public ProConMaterialBase
{
public:
  using Ptr = std::shared_ptr<MaterialInitialTransformations>;
  using Point = pcl_point_types::PointPcl;
  using Cloud = pcl_point_types::CloudPcl;

  Cloud::ConstPtr cloud_full;
  Cloud::ConstPtr cloud_edge;
  Cloud::ConstPtr cloud_edge_pot;
  Cloud::ConstPtr cloud_planar;
  Cloud::ConstPtr cloud_planar_pot;
  Eigen::VectorXd transform_global;
  Eigen::VectorXd transform_local;
  std_msgs::msg::Header::ConstSharedPtr header_in;

  bool will_map;

  void Log() override
  {
    std::cout << "count: " << count << std::endl <<
      "cloud_full->size: " << cloud_full->size() << std::endl <<
      "cloud_edge->size: " << cloud_edge->size() << std::endl <<
      "cloud_edge_pot->size: " << cloud_edge_pot->size() << std::endl <<
      "cloud_planar->size: " << cloud_planar->size() << std::endl <<
      "cloud_planar_pot->size: " << cloud_planar_pot->size() << std::endl <<
      "transform_global: " << transform_global.transpose() << std::endl <<
      "transform_local: " << transform_local.transpose() << std::endl <<
      "will_map: " << will_map << std::endl;
  }
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora

#endif  // MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_03_INIT_TRANSFORMATIONS_HPP_
