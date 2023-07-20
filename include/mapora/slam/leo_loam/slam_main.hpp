// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__SLAM_MAIN_HPP_
#define MAPORA__SLAM__LEO_LOAM__SLAM_MAIN_HPP_


#include <rclcpp/rclcpp.hpp>

#include "mapora/slam/leo_loam/producer_consumer_master.hpp"
#include "mapora/slam/leo_loam/feature_extractor.hpp"
#include "mapora/slam/leo_loam/init_trans_estimator.hpp"
#include "mapora/slam/leo_loam/mapper_occ.hpp"
#include "init_trans_estimator.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class SlamMain
{
public:
  SlamMain();
  void Start(rclcpp::Node & node,const MapperOcc::TypeCallbackTransform &callback_slam_transform);
  void Stop();

  ProducerConsumerMaster::TypePushTop get_push_top();

private:
  ProducerConsumerMaster::SharedPtr master_;
  FeatureExtractor::SharedPtr feature_extractor_;
  InitTransEstimator::SharedPtr init_trans_estimator_;
  MapperOcc::SharedPtr mapper_occ_;
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora


#endif  // MAPORA__SLAM__LEO_LOAM__SLAM_MAIN_HPP_
