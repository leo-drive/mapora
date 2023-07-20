// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM_HANDLER_HPP_
#define MAPORA__SLAM_HANDLER_HPP_

#include <folly/ProducerConsumerQueue.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <future>
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include "mapora/slam/leo_loam/slam_main.hpp"

namespace mapora
{
class SlamHandler
{
public:
  using SharedPtr = std::shared_ptr<SlamHandler>;
  using ConstSharedPtr = const std::shared_ptr<SlamHandler>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  explicit SlamHandler(
    std::shared_ptr<folly::ProducerConsumerQueue<PointCloud2::ConstSharedPtr>> & queue_clouds_raw,
    const slam::leo_loam::MapperOcc::TypeCallbackTransform &callback_slam_transform,
    rclcpp::Node & node);

  void WorkerSlam(
    std::shared_ptr<folly::ProducerConsumerQueue<PointCloud2::ConstSharedPtr>> & queue_clouds_raw,
    const slam::leo_loam::MapperOcc::TypeCallbackTransform &callback_slam_transform,
    rclcpp::Node & node);

private:
  std::atomic_bool keep_slamming_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_cloud_map_ndt_;

  std::future<void> future_worker_slam_;
};
}  // namespace mapora

#endif  // MAPORA__SLAM_HANDLER_HPP_
