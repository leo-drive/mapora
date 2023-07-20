// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__MAPPER_OCC_HPP_
#define MAPORA__SLAM__LEO_LOAM__MAPPER_OCC_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_search.h>

#include <random>
#include <memory>
#include <vector>
#include <cstdint>
#include <future>
#include <functional>

#include "mapora/utils.hpp"
#include "mapora/time_keeper_sequential.hpp"
#include "mapora/pcl_point_types.hpp"
#include "mapora/slam/leo_loam/trans_estimator.hpp"
#include "mapora/slam/leo_loam/occtree.hpp"
#include "mapora/slam/leo_loam/producer_consumer_master.hpp"
#include "mapora/slam/leo_loam/queue_management/material_03_init_transformations.hpp"
//  #include "MarkerStuff.h"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class MapperOcc
{
public:
  typedef std::shared_ptr<MapperOcc> SharedPtr;

  using Clock = time_keeper_sequential::TimeKeeperSequential::Clock;
  using TimePoint = time_keeper_sequential::TimeKeeperSequential::TimePoint;

  using Point = pcl_point_types::PointPcl;
  using Cloud = pcl_point_types::CloudPcl;

  using Kdtree = pcl::KdTreeFLANN<Point>;

  using TypeCallbackTransform =
    std::function<void (const geometry_msgs::msg::TransformStamped::ConstSharedPtr)>;

  explicit MapperOcc(
    rclcpp::Node & node, bool publish_things, TypeCallbackTransform callback_slam_transform);

  //  std::tuple<nav_msgs::Odometry,
  //    CloudPtrType,
  //    CloudPtrType> handler(const CloudConstPtrType &cloudEdgeIn,
  //                          const CloudConstPtrType &cloudPlanarIn,
  //                          const CloudConstPtrType &cloudFullIn,
  //                          const Eigen::VectorXd &odometry_in);

  ProConMaterialBase::Ptr Process(ProConMaterialBase::Ptr mat_in);

  ProducerConsumerMaster::FuncType GetMethod();

private:
  bool initialized_;
  rclcpp::Node & node_;
  rclcpp::Clock::SharedPtr clock_;
  bool publish_things_;
  TimePoint time_point_last_publish_box_;
  TimePoint time_point_last_publish_all_;
  int counter;
  float resolution_ds_edge;
  float resolution_ds_planar;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_correspondences;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudBox;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudAll;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudAllEdge;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudAllPlanar;

  TypeCallbackTransform callback_slam_transform_;

  typedef pcl::octree::OctreePointCloudSearch<Point> OctreeType;
  Occtree::Ptr occ_edge;
  Occtree::Ptr occ_planar;
  Occtree::Ptr occ_all_ds;
  Occtree::Ptr occ_search_all_ds;
  Occtree::Ptr occ_search_edge;
  Occtree::Ptr occ_search_planar;


  TransEstimator estimator;

  Eigen::VectorXd transformEstimatedLast;
  Eigen::VectorXd transformInLast;

  //    CloudPtrType cloudFullAll;


  void concatenateTransformation(
    const Eigen::VectorXd & transformIn,
    const Eigen::VectorXd & transformBefMapped,
    const Eigen::VectorXd & transformAftMapped,
    Eigen::VectorXd & transformTobeMapped);

  void pointAssociateToMap(
    const Point & pi, Point & po, const Eigen::VectorXd & transformTobeMapped);

  // Standard deviation values from a good motion
  const double R_AVG[3] = {1.5475, 1.5482, 1.5466};
  const double R_STD[3] = {1.5574, 1.5585, 1.5388};
  const double T_AVG[3] = {0.1589, 0.0602, 0.0293};
  const double T_STD[3] = {0.0818, 0.0436, 0.0232};
  const double AM_THRESH = 3.92;  // Abrupt motion threshold


  void publish_cloud(
    rclcpp::Time & time,
    Cloud::Ptr cloud_in,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);

  std::vector<Eigen::VectorXd> LevenbergAsync(
    const Cloud::Ptr & point,
    const Cloud::Ptr & cloudBoxFeat,
    const Kdtree::Ptr & kdTreeBoxFeat,
    int numKNeighbours,
    const TransEstimator & transEstimator,
    bool IsEdge) const;

  float random_color(uint64_t seed, float min, float max);

  Cloud::Ptr TransformCloud(const Cloud::ConstPtr cloudIn, const Eigen::VectorXd & transform6);

  static Eigen::Matrix3d make_mat_rot(Eigen::VectorXd Transform, const std::string & order)
  {
    Eigen::Matrix3d Z, Y, X, R;
    double Roll, Pitch, Yaw;
    Roll = Transform(0);
    Pitch = Transform(1);
    Yaw = Transform(2);
    Z << cos(Yaw), sin(Yaw), 0, -sin(Yaw), cos(Yaw), 0, 0, 0, 1;
    Y << cos(Pitch), 0, -sin(Pitch), 0, 1, 0, sin(Pitch), 0, cos(Pitch);
    X << 1, 0, 0, 0, cos(Roll), sin(Roll), 0, -sin(Roll), cos(Roll);
    if (order == "YXZ") {
      Eigen::Matrix3d temp;
      temp = X * Z;
      R = Y * temp;
    } else if (order == "ZXY") {
      Eigen::Matrix3d temp;
      temp = X * Y;
      R = Z * temp;
    }
    return R;
  }
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora

#endif  // MAPORA__SLAM__LEO_LOAM__MAPPER_OCC_HPP_
