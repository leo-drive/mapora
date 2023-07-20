// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__MAPORA_HPP_
#define MAPORA__MAPORA_HPP_

#include "mapora/visibility_control.hpp"
#include "mapora/points_provider_velodyne.hpp"
#include "mapora/transform_provider_sbg.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <folly/ProducerConsumerQueue.h>
#include <geometry_msgs/msg/pose_array.hpp>

#include "mapora/slam/slam_handler.hpp"

namespace mapora
{
class Mapora : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<Mapora>;
  using ConstSharedPtr = const std::shared_ptr<Mapora>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Points = points_provider::PointsProviderBase::Points;

  explicit Mapora(const rclcpp::NodeOptions & options);

  void process();

  // std::vector<points_provider::PointsProviderVelodyne::Points> clouds;

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_cloud_current_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_cloud_current_trans_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ptr_path_sbg_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_ptr_poses_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_ptr_poses_imu_origin_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_ptr_poses_slam_origin_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_ptr_poses_lidar_true_origin_;

  SlamHandler::SharedPtr slam_handler_;
  transform_provider::TransformProviderSbg::SharedPtr transform_provider_sbg_;
  points_provider::PointsProviderVelodyne::SharedPtr points_provider_velodyne_;

  int64_t index_pcap_start_;
  int64_t count_pcaps_to_process_;
  int64_t count_clouds_to_extract_max_;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_cloud_velodyne_raw_;

  void callback_cloud_velodyne_points_raw(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_in);

  static PointCloud2::SharedPtr points_to_cloud(
    const Points & points_bad, const std::string & frame_id);

  void callback_cloud_surround_out(const Points & points_surround);

  void callback_slam_transform(
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr & transform_slam);

  std::shared_ptr<folly::ProducerConsumerQueue<PointCloud2::ConstSharedPtr>> queue_clouds_raw_;
  std::shared_ptr<
    folly::ProducerConsumerQueue<geometry_msgs::msg::TransformStamped::ConstSharedPtr>>
  queue_slam_transforms_;
  std::vector<PointCloud2::SharedPtr> vector_clouds_raw_;

  std::vector<geometry_msgs::msg::TransformStamped> transforms_lidar_slam_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_imu_from_origin_;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> poses_imu_;
};
}  // namespace mapora

#endif  // MAPORA__MAPORA_HPP_
