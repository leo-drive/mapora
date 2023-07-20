// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/slam/slam_handler.hpp"
#include "mapora/pcl_utils.hpp"

#include <memory>
#include <future>

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <mapora/point_xyzi.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "mapora/slam/leo_loam/slam_main.hpp"

namespace mapora
{
SlamHandler::SlamHandler(
  std::shared_ptr<folly::ProducerConsumerQueue<PointCloud2::ConstSharedPtr>> & queue_clouds_raw,
  const slam::leo_loam::MapperOcc::TypeCallbackTransform & callback_slam_transform,
  rclcpp::Node & node)
: keep_slamming_{true},
  pub_ptr_cloud_map_ndt_{node.create_publisher<PointCloud2>("cloud_map_ndt", 10)},
  future_worker_slam_{std::async(
      std::launch::async,
      &SlamHandler::WorkerSlam,
      this,
      std::ref(queue_clouds_raw),
      std::cref(callback_slam_transform),
      std::ref(node))}
{
}

void SlamHandler::WorkerSlam(
  std::shared_ptr<folly::ProducerConsumerQueue<PointCloud2::ConstSharedPtr>> & queue_clouds_raw,
  const slam::leo_loam::MapperOcc::TypeCallbackTransform & callback_slam_transform,
  rclcpp::Node & node)
{
  //  using PointPcl = pcl::PointXYZI;
  //  using CloudPcl = pcl::PointCloud<PointPcl>;

  slam::leo_loam::SlamMain slam_main;
  slam_main.Start(node, callback_slam_transform);
  auto push_top = slam_main.get_push_top();
  int count_slam_handler_run = 0;
  while (keep_slamming_) {
    PointCloud2::ConstSharedPtr cloud_in;
    while (!queue_clouds_raw->read(cloud_in) && keep_slamming_) {
      // spin until we get a value
    }
    count_slam_handler_run++;
    std::cout << "count_slam_handler_run: " << count_slam_handler_run << std::endl;
    slam::leo_loam::MaterialPointsRaw::Ptr mat_ptr_in =
      std::make_shared<slam::leo_loam::MaterialPointsRaw>();
    mat_ptr_in->cloud_in = cloud_in;
    push_top(mat_ptr_in);
  }

  slam_main.Stop();
}
}  // namespace mapora
