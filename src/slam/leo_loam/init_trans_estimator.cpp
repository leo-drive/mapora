// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/slam/leo_loam/init_trans_estimator.hpp"
#include "mapora/pcl_utils.hpp"

#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <execution>
#include <vector>
#include <memory>
#include <utility>
#include <bitset>
#include <algorithm>

#include <tbb/iterators.h>

namespace mapora
{
namespace slam
{
namespace leo_loam
{
using Point = InitTransEstimator::Point;
using Cloud = InitTransEstimator::Cloud;

InitTransEstimator::InitTransEstimator(rclcpp::Node & node)
: pub_markers_correspondences{node.create_publisher<visualization_msgs::msg::MarkerArray>(
      "markers_correspondences_mapper_torch", 1)},
  pub_odom_odom_{node.create_publisher<nav_msgs::msg::Odometry>("odometry_odom", 1)},
  pub_cloud_imu_corrected_{
    node.create_publisher<sensor_msgs::msg::PointCloud2>("cloud_imu_corrected", 1)}
{
  cloud_edge_pot_prev_.reset(new Cloud);
  cloud_planar_pot_prev_.reset(new Cloud);

  kdtree_edge_pot_prev_.reset(new Kdtree);
  kdtree_planar_pot_prev_.reset(new Kdtree);

  initTrans = std::make_shared<InitTransHelper>();

  frameCount = 1;
  transform.setZero(6);
  transform_global.setZero(6);
}

ProConMaterialBase::Ptr InitTransEstimator::Process(ProConMaterialBase::Ptr mat_in)
{
  auto mat_feat = std::dynamic_pointer_cast<MaterialFeatures>(mat_in);
  if (!mat_feat) {
    throw std::runtime_error("Cannot cast ProConMaterialMother to MaterialFeatures");
  }

  Cloud::ConstPtr cloud_full_in = mat_feat->cloud_full;
  Cloud::ConstPtr cloud_edge_in = mat_feat->cloud_edge;
  Cloud::ConstPtr cloud_edge_pot_in = mat_feat->cloud_edge_pot;
  Cloud::ConstPtr cloud_planar_in = mat_feat->cloud_planar;
  Cloud::ConstPtr cloud_planar_pot_in = mat_feat->cloud_planar_pot;

  auto ReplicateCloud = [](const Cloud::ConstPtr & cloud) -> Cloud::Ptr {
      Cloud::Ptr cloud_copy(new Cloud);
      cloud_copy->points.resize(cloud->points.size());
      std::copy(
        std::execution::par, cloud->points.begin(), cloud->points.end(),
        cloud_copy->points.begin());
      return cloud_copy;
    };

  Cloud::Ptr cloud_full = ReplicateCloud(cloud_full_in);
  Cloud::Ptr cloud_edge = ReplicateCloud(cloud_edge_in);
  Cloud::Ptr cloud_edge_pot = ReplicateCloud(cloud_edge_pot_in);
  Cloud::Ptr cloud_planar = ReplicateCloud(cloud_planar_in);
  Cloud::Ptr cloud_planar_pot = ReplicateCloud(cloud_planar_pot_in);

//  auto deg_to_rad = [](const double & deg) {return deg * M_PI / 180.0;};

//  const auto Publish = [](
//    const Cloud::ConstPtr & cloud,
//    rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & pub,
//    const rclcpp::Time & time,
//    const std::string & frame) {
//      sensor_msgs::msg::PointCloud2::SharedPtr msg =
//        PclUtils::cloud_pcl_to_pointcloud2(cloud, frame);
//      msg->header.set__stamp(time);
//      pub.publish(*msg);
//    };

  //  if (mat_feat->use_imu) {
  //    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  //    Eigen::Matrix3d mat_rotation = Eigen::Matrix3d(
  //      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
  //      Eigen::AngleAxisd(deg_to_rad(mat_feat->Pitch), Eigen::Vector3d::UnitY()) *
  //      Eigen::AngleAxisd(deg_to_rad(mat_feat->Roll), Eigen::Vector3d::UnitX()));
  //    T.block(0, 0, 3, 3) = mat_rotation;
  //
  //    Eigen::Affine3d affine(T);
  //    affine = affine.inverse();
  //
  //    auto TransformCloud =
  //      [&Publish](const Cloud::ConstPtr & cloud, const Eigen::Affine3d & affine) -> Cloud::Ptr {
  //      Cloud::Ptr cloud_transformed(new Cloud);
  //      cloud_transformed->points.resize(cloud->points.size());
  //
  //      auto TransformPoint = [](const Point & point_in, const Eigen::Affine3d & affine) {
  //        return pcl::transformPoint(point_in, affine);
  //      };
  //
  //      using std::placeholders::_1;
  //      std::transform(
  //        std::execution::par,
  //        cloud->points.begin(),
  //        cloud->points.end(),
  //        cloud_transformed->points.begin(),
  //        std::bind(TransformPoint, _1, affine));
  //      return cloud_transformed;
  //    };
  //
  //    cloud_full = TransformCloud(cloud_full, affine);
  //    cloud_edge = TransformCloud(cloud_edge, affine);
  //    cloud_edge_pot = TransformCloud(cloud_edge_pot, affine);
  //    cloud_planar = TransformCloud(cloud_planar, affine);
  //    cloud_planar_pot = TransformCloud(cloud_planar_pot, affine);
  //    Publish(cloud_full, *pub_cloud_imu_corrected_, mat_feat->header_in->stamp, "map");
  //  }

  if (!systemInited) {
    *cloud_edge_pot_prev_ = *cloud_edge_pot;
    *cloud_planar_pot_prev_ = *cloud_planar_pot;
    systemInited = true;

    // Crashes if map is broken so badly
    // Todo: Handle this with messagebox messages.
    if (cloud_edge_pot_prev_->points.size() < 5) {
      throw std::logic_error("Odometry 1st cloudEdgePotLast->points.size()<5.");
    }
    if (cloud_planar_pot_prev_->points.size() < 5) {
      throw std::logic_error("Odometry 1st cloudPlanarPotLast->points.size()<5.");
    }

    kdtree_edge_pot_prev_->setInputCloud(cloud_edge_pot_prev_);
    kdtree_planar_pot_prev_->setInputCloud(cloud_planar_pot_prev_);

    MaterialInitialTransformations::Ptr mat_odometry =
      std::make_shared<MaterialInitialTransformations>();
    mat_odometry->cloud_full = cloud_full;
    mat_odometry->cloud_edge = cloud_edge;
    mat_odometry->cloud_edge_pot = cloud_edge_pot;
    mat_odometry->cloud_planar = cloud_planar;
    mat_odometry->cloud_planar_pot = cloud_planar_pot;
    mat_odometry->transform_global = transform_global;
    mat_odometry->transform_local = transform;
    mat_odometry->will_map = true;
    mat_odometry->header_in = mat_feat->header_in;
    return mat_odometry;
  }

  if (cloud_edge_pot_prev_->points.size() > 10 && cloud_planar_pot->points.size() > 100) {
    //////////////////////////////////////////////////////////////
    ///////////////// LM Algorithm
    //////////////////////////////////////////////////////////////

    std::vector<std::vector<Point>> allEdgeNeighbors(cloud_edge->points.size());
    std::vector<std::vector<Point>> allPlanarNeighbors(cloud_planar->points.size());

    //    bool isDegenerate{false};
    int iter_count_total{0};
    for (int iterCount = 0; iterCount < 25; iterCount++) {
      iter_count_total = iterCount;


      //////////////////////////////////////////////////////////////
      ///////////////// CORNER
      //////////////////////////////////////////////////////////////

      const size_t count_cloud_edge_points = cloud_edge->points.size();

      std::vector<Eigen::VectorXd> a_lines_edge(count_cloud_edge_points);
      std::vector<std::bitset<1>> should_use_a_lines_edge(count_cloud_edge_points);

      std::transform(
        std::execution::par,
        cloud_edge->points.begin(),
        cloud_edge->points.end(),
        tbb::make_zip_iterator(a_lines_edge.begin(), should_use_a_lines_edge.begin()),
        [this, iterCount](const Point & point) {
          Point pointSel = TransformToStart(point);
          std::vector<Point> neighbors;

          if (iterCount % 5 == 0) {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtree_edge_pot_prev_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[0] < 25) {
              std::vector<int> pointSearchInd2;
              kdtree_edge_pot_prev_->radiusSearch(pointSel, 5, pointSearchInd2, pointSearchSqDis);
              initTrans->NeighborSelectionEdge(
                pointSearchInd[0], cloud_edge_pot_prev_, pointSearchInd2, neighbors);
            }
          }
          Eigen::VectorXd aLine(7);
          bool thing =
          initTrans->PointToLine(aLine, transform, point, pointSel, neighbors, iterCount);
          return std::make_tuple(aLine, std::bitset<1>{thing});
        });


      //////////////////////////////////////////////////////////////
      ///////////////// SURFACE
      //////////////////////////////////////////////////////////////

      const size_t count_cloud_planar_points = cloud_planar->points.size();

      std::vector<Eigen::VectorXd> a_lines_planar(count_cloud_planar_points);
      std::vector<std::bitset<1>> should_use_a_lines_planar(count_cloud_planar_points);

      std::transform(
        std::execution::par,
        cloud_planar->points.begin(),
        cloud_planar->points.end(),
        tbb::make_zip_iterator(a_lines_planar.begin(), should_use_a_lines_planar.begin()),
        [this, iterCount](const Point & point) {
          std::vector<int> pointSearchInd;
          std::vector<float> pointSearchSqDis;
          Point pointSel = TransformToStart(point);
          std::vector<Point> neighbors;

          if (iterCount % 5 == 0) {
            kdtree_planar_pot_prev_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[0] < 25) {
              std::vector<int> pointSearchInd2;
              kdtree_planar_pot_prev_->radiusSearch(pointSel, 2, pointSearchInd2, pointSearchSqDis);
              initTrans->NeighborSelectionFlat(
                pointSearchInd[0], cloud_planar_pot_prev_, pointSearchInd2, neighbors);
            }
          }

          Eigen::VectorXd Aline(7);
          bool thing =
          initTrans->PointToPlane(Aline, transform, point, pointSel, neighbors, iterCount);
          return std::make_tuple(Aline, std::bitset<1>{thing});
        });


      std::vector<Eigen::VectorXd> a_lines;

      for_each(
        std::execution::seq,
        tbb::make_zip_iterator(a_lines_edge.begin(), should_use_a_lines_edge.begin()),
        tbb::make_zip_iterator(a_lines_edge.end(), should_use_a_lines_edge.end()),
        [&a_lines](const std::tuple<Eigen::VectorXd, std::bitset<1>> & aline_and_should_use) {
          Eigen::VectorXd a_line = std::get<0>(aline_and_should_use);
          bool should_use = std::get<1>(aline_and_should_use)[0];
          if (should_use) {a_lines.push_back(a_line);}
        });

      for_each(
        std::execution::seq,
        tbb::make_zip_iterator(a_lines_planar.begin(), should_use_a_lines_planar.begin()),
        tbb::make_zip_iterator(a_lines_planar.end(), should_use_a_lines_planar.end()),
        [&a_lines](const std::tuple<Eigen::VectorXd, std::bitset<1>> & aline_and_should_use) {
          Eigen::VectorXd a_line = std::get<0>(aline_and_should_use);
          bool should_use = std::get<1>(aline_and_should_use)[0];
          if (should_use) {a_lines.push_back(a_line);}
        });

      if (initTrans->Compute(a_lines, transform)) {break;}
      //      time_keeper_lm.PrintTimes();
    }

//    std::cout << "LM iterations count: " << iter_count_total << std::endl;
  }

  float rx, ry, rz, tx, ty, tz;
  AccumulateRotation(
    transform_global(0),
    transform_global(1),
    transform_global(2),
    -transform(0),
    -transform(1),
    -transform(2),
    rx,
    ry,
    rz);

  double x1 = cos(rz) * (transform(3)) - sin(rz) * (transform(4));
  double y1 = sin(rz) * (transform(3)) + cos(rz) * (transform(4));
  double z1 = transform(5);

  double x2 = x1;
  double y2 = cos(rx) * y1 - sin(rx) * z1;
  double z2 = sin(rx) * y1 + cos(rx) * z1;

  tx = transform_global(3) - (cos(ry) * x2 + sin(ry) * z2);
  ty = transform_global(4) - y2;
  tz = transform_global(5) - (-sin(ry) * x2 + cos(ry) * z2);

  // Global odometry
  transform_global(0) = rx;
  transform_global(1) = ry;
  transform_global(2) = rz;
  transform_global(3) = tx;
  transform_global(4) = ty;
  transform_global(5) = tz;

  *cloud_edge_pot_prev_ = *cloud_edge_pot;
  *cloud_planar_pot_prev_ = *cloud_planar_pot;

  for (auto & point : cloud_edge_pot_prev_->points) {
    point = TransformToEnd(point);
  }
  for (auto & point : cloud_planar_pot_prev_->points) {
    point = TransformToEnd(point);
  }

  frameCount++;

  if (cloud_edge_pot_prev_->points.size() < 5) {
    throw std::logic_error("Odometry cloudEdgePotLast->points.size()<5.");
  }
  if (cloud_planar_pot_prev_->points.size() < 5) {
    throw std::logic_error("Odometry cloudPlanarPotLast->points.size()<5.");
  }

  if (cloud_edge_pot_prev_->points.size() > 10 && cloud_planar_pot_prev_->points.size() > 100) {
    kdtree_edge_pot_prev_->setInputCloud(cloud_edge_pot_prev_);
    kdtree_planar_pot_prev_->setInputCloud(cloud_planar_pot_prev_);
  }

  // will map every 2 times
  bool willMap = false;
  if (frameCount >= 0) {
    frameCount = 0;
    willMap = true;
  }
  willMap = true;

  tf2::Quaternion geoQuat;
  geoQuat.setRPY(transform_global(0), transform_global(1), transform_global(2));
  //  geometry_msgs::msg::Quaternion geoQuat = tf2_ros::createQuaternionMsgFromRollPitchYaw(
  //    transform_global(0), transform_global(1), transform_global(2));

  nav_msgs::msg::Odometry msg_odom;
  msg_odom.header.frame_id = "map";
  msg_odom.child_frame_id = "torch_odom";

  msg_odom.header.stamp = mat_feat->header_in->stamp;
  msg_odom.pose.pose.orientation.x = geoQuat.x();
  msg_odom.pose.pose.orientation.y = geoQuat.y();
  msg_odom.pose.pose.orientation.z = geoQuat.z();
  msg_odom.pose.pose.orientation.w = geoQuat.w();
  msg_odom.pose.pose.position.x = transform_global(3);
  msg_odom.pose.pose.position.y = transform_global(4);
  msg_odom.pose.pose.position.z = transform_global(5);
  pub_odom_odom_->publish(msg_odom);

  //  time_keeper.PrintTimes();

  MaterialInitialTransformations::Ptr mat_odometry =
    std::make_shared<MaterialInitialTransformations>();
  mat_odometry->cloud_full = cloud_full;
  mat_odometry->cloud_edge = cloud_edge;
  mat_odometry->cloud_edge_pot = cloud_edge_pot;
  mat_odometry->cloud_planar = cloud_planar;
  mat_odometry->cloud_planar_pot = cloud_planar_pot;
  mat_odometry->transform_global = transform_global;
  mat_odometry->transform_local = transform;
  mat_odometry->will_map = willMap;
  mat_odometry->header_in = mat_feat->header_in;
  return mat_odometry;
}

Point InitTransEstimator::TransformToStart(const Point & pi)
{
  float s = 10 * (pi.intensity - static_cast<int>(pi.intensity));

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  float x1 = cos(rz) * (pi.x - tx) + sin(rz) * (pi.y - ty);
  float y1 = -sin(rz) * (pi.x - tx) + cos(rz) * (pi.y - ty);
  float z1 = (pi.z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  Point po;
  po.x = cos(ry) * x2 - sin(ry) * z2;
  po.y = y2;
  po.z = sin(ry) * x2 + cos(ry) * z2;
  po.intensity = pi.intensity;
  return po;
}

Point InitTransEstimator::TransformToEnd(const Point & pi)
{
  float s = 10 * (pi.intensity - static_cast<int>(pi.intensity));

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  float x1 = cos(rz) * (pi.x - tx) + sin(rz) * (pi.y - ty);
  float y1 = -sin(rz) * (pi.x - tx) + cos(rz) * (pi.y - ty);
  float z1 = (pi.z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  rx = transform[0];
  ry = transform[1];
  rz = transform[2];
  tx = transform[3];
  ty = transform[4];
  tz = transform[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
  float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
  float z6 = z5 + tz;

  Point po;
  po.x = x6;
  po.y = y6;
  po.z = z6;
  po.intensity = static_cast<int>(pi.intensity);
  return po;
}

void InitTransEstimator::AccumulateRotation(
  float ROLL,
  float PITCH,
  float YAW,
  float roll,
  float pitch,
  float yaw,
  float & ox,
  float & oy,
  float & oz)
{
  double srx = cos(roll) * cos(ROLL) * sin(pitch) * sin(YAW) - cos(ROLL) * cos(YAW) * sin(roll) -
    cos(roll) * cos(pitch) * sin(ROLL);
  ox = -asin(srx);

  double srycrx =
    sin(roll) * (cos(PITCH) * sin(YAW) - cos(YAW) * sin(ROLL) * sin(PITCH)) +
    cos(roll) * sin(pitch) * (cos(PITCH) * cos(YAW) + sin(ROLL) * sin(PITCH) * sin(YAW)) +
    cos(roll) * cos(pitch) * cos(ROLL) * sin(PITCH);
  double crycrx =
    cos(roll) * cos(pitch) * cos(ROLL) * cos(PITCH) -
    cos(roll) * sin(pitch) * (cos(YAW) * sin(PITCH) - cos(PITCH) * sin(ROLL) * sin(YAW)) -
    sin(roll) * (sin(PITCH) * sin(YAW) + cos(PITCH) * cos(YAW) * sin(ROLL));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  double srzcrx =
    sin(ROLL) * (cos(yaw) * sin(pitch) - cos(pitch) * sin(roll) * sin(yaw)) +
    cos(ROLL) * sin(YAW) * (cos(pitch) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw)) +
    cos(roll) * cos(ROLL) * cos(YAW) * sin(yaw);
  double crzcrx =
    cos(roll) * cos(yaw) * cos(ROLL) * cos(YAW) -
    cos(ROLL) * sin(YAW) * (cos(pitch) * sin(yaw) - cos(yaw) * sin(roll) * sin(pitch)) -
    sin(ROLL) * (sin(pitch) * sin(yaw) + cos(pitch) * cos(yaw) * sin(roll));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

ProducerConsumerMaster::FuncType InitTransEstimator::GetMethod()
{
  using std::placeholders::_1;
  return std::bind(&InitTransEstimator::Process, this, _1);
}

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora
