// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/slam/leo_loam/slam_main.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
SlamMain::SlamMain() {}
void SlamMain::Start(
  rclcpp::Node & node, const MapperOcc::TypeCallbackTransform & callback_slam_transform)
{
  master_ = std::make_unique<ProducerConsumerMaster>(99999);

  master_->set_debug_queue_size(false);
  master_->set_debug_results(false);
  master_->set_debug_timing(false);
  master_->set_debug_detail(false);

  feature_extractor_ = std::make_shared<FeatureExtractor>(node);
  auto func = feature_extractor_->GetMethod();
  master_->AddWorker(func, "FeatureExtractor", false);

  init_trans_estimator_ = std::make_shared<InitTransEstimator>(node);
  auto func2 = init_trans_estimator_->GetMethod();
  master_->AddWorker(func2, "InitTransEstimator", false);

  mapper_occ_ = std::make_shared<MapperOcc>(node, true, callback_slam_transform);
  auto func3 = mapper_occ_->GetMethod();
  master_->AddWorker(func3, "MapperOcc", true);

  master_->Start();
}

void SlamMain::Stop() {master_->KillAll();}

ProducerConsumerMaster::TypePushTop SlamMain::get_push_top() {return master_->get_push_top();}

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora
