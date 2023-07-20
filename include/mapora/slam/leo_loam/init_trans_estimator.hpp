// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__INIT_TRANS_ESTIMATOR_HPP_
#define MAPORA__SLAM__LEO_LOAM__INIT_TRANS_ESTIMATOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

#include "mapora/pcl_point_types.hpp"
#include "mapora/slam/leo_loam/init_trans_helper.hpp"

#include "producer_consumer_master.hpp"
#include "mapora/slam/leo_loam/queue_management/material_02_features.hpp"
#include "mapora/slam/leo_loam/queue_management/material_03_init_transformations.hpp"
// #include "MarkerStuff.h"
// #include "PclStuff.h"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class InitTransEstimator
{
public:
  using SharedPtr = std::shared_ptr<InitTransEstimator>;
  using Point = pcl_point_types::PointPcl;
  using Cloud = pcl_point_types::CloudPcl;

  using Kdtree = pcl::KdTreeFLANN<Point>;

  explicit InitTransEstimator(rclcpp::Node & node);

  ProducerConsumerMaster::FuncType GetMethod();

private:
  ProConMaterialBase::Ptr Process(ProConMaterialBase::Ptr mat_in);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_correspondences;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_imu_corrected_;

  bool systemInited = false;

  Kdtree::Ptr kdtree_edge_pot_prev_;
  Kdtree::Ptr kdtree_planar_pot_prev_;

  Cloud::Ptr cloud_edge_pot_prev_;
  Cloud::Ptr cloud_planar_pot_prev_;

  InitTransHelper::Ptr initTrans;

  Eigen::VectorXd transform;
  Eigen::VectorXd transform_global;

  int frameCount;

  Point TransformToEnd(const Point & pi);

  Point TransformToStart(const Point & pi);

  void AccumulateRotation(
    float ROLL,
    float PITCH,
    float YAW,
    float roll,
    float pitch,
    float yaw,
    float & ox,
    float & oy,
    float & oz);
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora

#endif  // MAPORA__SLAM__LEO_LOAM__INIT_TRANS_ESTIMATOR_HPP_
