// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_01_POINTS_RAW_HPP_
#define MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_01_POINTS_RAW_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

#include "mapora/slam/leo_loam/producer_consumer_master.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class MaterialPointsRaw : public ProConMaterialBase
{
public:
  using Ptr = std::shared_ptr<MaterialPointsRaw>;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_in;

  void Log() override
  {
    std::cout << "count: " << count << std::endl;
    std::cout << "points_count: " << cloud_in->height * cloud_in->width << std::endl;
  }
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora

#endif  // MAPORA__SLAM__LEO_LOAM__QUEUE_MANAGEMENT__MATERIAL_01_POINTS_RAW_HPP_
