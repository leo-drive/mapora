// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/slam/leo_loam/mapper_occ.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>

#include <tbb/iterators.h>

#include <execution>
#include <thread>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include <utility>

#include "mapora/slam/leo_loam/trans_estimator.hpp"
#include "mapora/pcl_utils.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
using Point = MapperOcc::Point;
using Cloud = MapperOcc::Cloud;

using Clock = MapperOcc::Clock;
using TimePoint = MapperOcc::TimePoint;
using TimeKeeperSequential = time_keeper_sequential::TimeKeeperSequential;

MapperOcc::MapperOcc(
  rclcpp::Node & node, bool publish_things, TypeCallbackTransform callback_slam_transform)
: initialized_(false),
  node_{node},
  clock_{node.get_clock()},
  publish_things_(publish_things),
  time_point_last_publish_box_(Clock::from_time_t(0)),
  time_point_last_publish_all_(Clock::from_time_t(0)),
  counter(0),
  resolution_ds_edge(0.2f),
  resolution_ds_planar(0.4f),
  pub_markers_correspondences{node.create_publisher<visualization_msgs::msg::MarkerArray>(
      "markers_correspondences_mapper_torch", 1)},
  pubOdomAftMapped{node.create_publisher<nav_msgs::msg::Odometry>("odometry_mapper", 1)},
  pubCloudBox{node.create_publisher<sensor_msgs::msg::PointCloud2>("cloud_torch_box", 1)},
  pubCloudAll{node.create_publisher<sensor_msgs::msg::PointCloud2>("cloud_torch_all", 1)},
  pubCloudAllEdge{node.create_publisher<sensor_msgs::msg::PointCloud2>("cloud_torch_all_edge", 1)},
  pubCloudAllPlanar{
    node.create_publisher<sensor_msgs::msg::PointCloud2>("cloud_torch_all_planar", 1)},
  callback_slam_transform_{callback_slam_transform}
{
  transformInLast.setZero(6);
  transformEstimatedLast.setZero(6);

  occ_edge = std::make_shared<Occtree>(resolution_ds_edge);
  occ_planar = std::make_shared<Occtree>(resolution_ds_planar);
  float resolution_ds_vis = 0.25;
  occ_all_ds = std::make_shared<Occtree>(resolution_ds_vis);
  occ_search_edge = std::make_shared<Occtree>(100);
  occ_search_planar = std::make_shared<Occtree>(100);
  occ_search_all_ds = std::make_shared<Occtree>(100);
}

ProConMaterialBase::Ptr MapperOcc::Process(ProConMaterialBase::Ptr mat_in)
{
  auto mat_odom = std::dynamic_pointer_cast<MaterialInitialTransformations>(mat_in);
  if (!mat_odom) {
    throw std::runtime_error("Cannot cast ProConMaterialMother to MaterialInitialTransformations");
  }

  if (!mat_odom->will_map) {
    ProConMaterialBase::Ptr mat_dummy = std::make_shared<ProConMaterialBase>();
    return mat_dummy;
    //    throw std::logic_error("MapperOcc won't map.");
  }
  ProConMaterialBase::Ptr mat_dummy = std::make_shared<ProConMaterialBase>();
  //  return mat_dummy;

  TimeKeeperSequential time_keeper("MapperOcc");
  time_keeper.add_time_point("Start");

  Cloud::ConstPtr cloudEdgeIn = mat_odom->cloud_edge_pot;
  Cloud::ConstPtr cloudPlanarIn = mat_odom->cloud_planar_pot;
  Cloud::ConstPtr cloudFullIn = mat_odom->cloud_full;
  Eigen::VectorXd transformIn = mat_odom->transform_global;

  rclcpp::Time timeStartTime = clock_->now();

  Cloud::Ptr cloudDsEdge = PclUtils::downsample(cloudEdgeIn, resolution_ds_edge);

  Cloud::Ptr cloudDsPlanar = PclUtils::downsample(cloudPlanarIn, resolution_ds_planar);

  time_keeper.add_time_point("Downsample");

  if (!initialized_) {
    occ_edge->AddPointsFromCloud(cloudDsEdge);
    occ_search_edge->AddPointsFromCloud(cloudDsEdge);
    occ_all_ds->AddPointsFromCloud(cloudFullIn);

    occ_planar->AddPointsFromCloud(cloudDsPlanar);
    occ_search_planar->AddPointsFromCloud(cloudDsPlanar);


    nav_msgs::msg::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "torch";

    odomAftMapped.header.stamp = mat_odom->header_in->stamp;
    odomAftMapped.pose.pose.orientation.x = 0;
    odomAftMapped.pose.pose.orientation.y = 0;
    odomAftMapped.pose.pose.orientation.z = 0;
    odomAftMapped.pose.pose.orientation.w = 1;
    odomAftMapped.pose.pose.position.x = 0;
    odomAftMapped.pose.pose.position.y = 0;
    odomAftMapped.pose.pose.position.z = 0;
    pubOdomAftMapped->publish(odomAftMapped);


    initialized_ = true;
    // publish initial potential points
    if (publish_things_) {
      publish_cloud(timeStartTime, occ_edge->cloud, pubCloudAllEdge);
      publish_cloud(timeStartTime, occ_planar->cloud, pubCloudAllPlanar);
    }
    Eigen::VectorXd trans(6);
    trans.setZero(6);
    counter++;
    throw std::logic_error("MapperOcc Initialized.");
  }

  if (publish_things_) {
    publish_cloud(timeStartTime, occ_edge->cloud, pubCloudAllEdge);
    publish_cloud(timeStartTime, occ_planar->cloud, pubCloudAllPlanar);
  }
  Eigen::VectorXd transformEstimated;
  concatenateTransformation(
    transformIn, transformInLast, transformEstimatedLast, transformEstimated);

  Eigen::VectorXd tbmmmm(6);
  tbmmmm << transformEstimated[0], transformEstimated[1], transformEstimated[2],
    transformEstimated[3], transformEstimated[4], transformEstimated[5];
  estimator.setTransformation(tbmmmm);

  Point pt_search;
  pt_search.x = static_cast<float>(transformEstimated(3));
  pt_search.y = static_cast<float>(transformEstimated(4));
  pt_search.z = static_cast<float>(transformEstimated(5));

  auto get_kdtree = [](const Occtree::Ptr & occtree, const Point & pt_search, float distance) {
      TimeKeeperSequential time_keeper_kd("kdtree_boi");
      time_keeper_kd.add_time_point("Start");

      Cloud::Ptr cloudBox = occtree->Box(pt_search, distance);
      time_keeper_kd.add_time_point("Octree");
      if (cloudBox->points.size() < 5) {
        throw std::logic_error("Mapper 1st cloudBox->points.size()<5.");
      }
      Kdtree::Ptr kdTreeBox(new Kdtree);
      kdTreeBox->setInputCloud(cloudBox);
      time_keeper_kd.add_time_point("Kdtree");
//      time_keeper_kd.print_times();
      return std::make_tuple(cloudBox, kdTreeBox);
    };

  std::future<std::tuple<Cloud::Ptr, Kdtree::Ptr>> future_edge =
    std::async(std::launch::async, get_kdtree, occ_search_edge, pt_search, 70);
  std::future<std::tuple<Cloud::Ptr, Kdtree::Ptr>> future_planar =
    std::async(std::launch::async, get_kdtree, occ_search_planar, pt_search, 70);

  Cloud::Ptr cloudBoxEdge;
  Kdtree::Ptr kdTreeBoxEdge;
  std::tie(cloudBoxEdge, kdTreeBoxEdge) = future_edge.get();

  Cloud::Ptr cloudBoxPlanar;
  Kdtree::Ptr kdTreeBoxPlanar;
  std::tie(cloudBoxPlanar, kdTreeBoxPlanar) = future_planar.get();

  time_keeper.add_time_point("Kdtrees");
  //  bool isDegenerate{false};


  // Actual Mapping
  int num_closest_neighbours = 5;

  //////////////////////////////////////////////////
  ///////////////////////// LEVENBERG-MARQUARDT
  //////////////////////////////////////////////////
  // Levenberg-Marquardt algorithm with 10 iterations
  TimeKeeperSequential time_keeper_lm("LM MapperOcc");
  for (int iterCount = 0; iterCount < 10; iterCount++) {
    std::string str_iter{"Iter #" + std::to_string(iterCount) + " "};
    time_keeper_lm.add_time_point(str_iter + "Start");
    //    auto markerLineListCorner = MarkerStuff::lineList("map", 30, 0, 1, 1);
    //    auto markerLineListPlanar = MarkerStuff::lineList("map", 31, 1, 1, 0);

    //////////////////////////////////////////////////
    ///////////////////////// CORNER
    //////////////////////////////////////////////////

    std::vector<Eigen::VectorXd> a_lines_corner = MapperOcc::LevenbergAsync(
      cloudDsEdge, cloudBoxEdge, kdTreeBoxEdge, num_closest_neighbours, estimator, true);

    time_keeper_lm.add_time_point(str_iter + "Edges");

    //////////////////////////////////////////////////
    ///////////////////////// SURFACE
    //////////////////////////////////////////////////

    std::vector<Eigen::VectorXd> a_lines_planar = MapperOcc::LevenbergAsync(
      cloudDsPlanar, cloudBoxPlanar, kdTreeBoxPlanar, num_closest_neighbours, estimator, false);

    time_keeper_lm.add_time_point(str_iter + "Planars");

    std::vector<Eigen::VectorXd> a_lines_all;
    for (const auto & aline : a_lines_corner) {
      a_lines_all.push_back(aline);
    }
    for (const auto & aline : a_lines_planar) {
      a_lines_all.push_back(aline);
    }

    //    std::vector<Eigen::VectorXd> a_lines_all(
    //        a_lines_corner.size() + a_lines_planar.size());
    //
    //    thrust::merge(thrust::system::cpp::par,
    //                  a_lines_corner.begin(),
    //                  a_lines_corner.end(),
    //                  a_lines_planar.begin(),
    //                  a_lines_planar.end(),
    //                  a_lines_all.begin());

    //////////////////////////////////////////////////
    ///////////////////////// SOLVE
    //////////////////////////////////////////////////
    if (a_lines_all.size() < 20) {continue;}

    Eigen::VectorXd X(6);

    bool converged = estimator.Compute(a_lines_all, X);

    time_keeper_lm.add_time_point(str_iter + "Solve");

    //    if (publish_things_) {
    //      visualization_msgs::msg::MarkerArray markersCorrespondenceLines;
    //      markersCorrespondenceLines.markers.clear();
    //      markersCorrespondenceLines.markers.push_back(markerLineListCorner);
    //      markersCorrespondenceLines.markers.push_back(markerLineListPlanar);
    //      //      pub_markers_correspondences.publish(markersCorrespondenceLines);
    //    }
    if (converged
      //      ||      watch2.ElapsedMilliSeconds() > 150
    )
    {
      break;
    }
  }
  //  time_keeper_lm.PrintTimes();

  time_keeper.add_time_point("LM");

  Cloud::Ptr cloudFullTrans(new Cloud());
  Cloud::Ptr cloudEdgeTrans(new Cloud());
  Cloud::Ptr cloudPlanarTrans(new Cloud());

  transformEstimated = estimator.getTransformation();

  transformInLast = transformIn;

  transformEstimatedLast = transformEstimated;

  // apply refined global transformation to initial cloud
  cloudFullTrans = TransformCloud(cloudFullIn, transformEstimated);
  // Transform new potential points with final transformation
  cloudEdgeTrans = TransformCloud(cloudDsEdge, transformEstimated);
  cloudPlanarTrans = TransformCloud(cloudDsPlanar, transformEstimated);

  time_keeper.add_time_point("Transform");

  //    auto cloudFullEdgeTrans = TransformCloud(cloudEdgeIn, transformEstimated);
  //    auto cloudFullPlanarTrans = TransformCloud(cloudPlanarIn, transformEstimated);

  //    publish_cloud(timeStart, cloudFullEdgeTrans, pubCloudEdgeLast);
  //    publish_cloud(timeStart, cloudFullPlanarTrans, pubCloudPlanarLast);
  //    publish_cloud(timeStart, cloudFullTrans, pubCloudFullLast);


  // Add transformed potential points to octrees
  for (const auto & point : cloudEdgeTrans->points) {
    if (!occ_edge->octree->isVoxelOccupiedAtPoint(point)) {
      occ_edge->AddPoint(point);
      occ_search_edge->AddPoint(point);
    }
  }
  for (const auto & point : cloudPlanarTrans->points) {
    if (!occ_planar->octree->isVoxelOccupiedAtPoint(point)) {
      occ_planar->AddPoint(point);
      occ_search_planar->AddPoint(point);
    }
  }
  time_keeper.add_time_point("Octrees");

  for (const auto & point : cloudEdgeTrans->points) {
    if (!occ_all_ds->octree->isVoxelOccupiedAtPoint(point)) {
      occ_all_ds->AddPoint(point);
      occ_search_all_ds->AddPoint(point);
    }
  }
  for (const auto & point : cloudPlanarTrans->points) {
    if (!occ_all_ds->octree->isVoxelOccupiedAtPoint(point)) {
      occ_all_ds->AddPoint(point);
      occ_search_all_ds->AddPoint(point);
    }
  }
  time_keeper.add_time_point("Octrees Visual");


  // search point in final sensor position for visualization
  //  if (
  //    TimeKeeperSequential::millis_passed_since(time_point_last_publish_box_) > 1000 &&
  //    publish_things_)
  {
    // box search to fill final cloud
    Point pt_search;
    pt_search.x = static_cast<float>(transformEstimated(3));
    pt_search.y = static_cast<float>(transformEstimated(4));
    pt_search.z = static_cast<float>(transformEstimated(5));
    Cloud::Ptr cloudFinal(new Cloud());
    Cloud::Ptr boxEdge = occ_search_edge->Box(pt_search, 100);
    Cloud::Ptr boxPlanar = occ_search_planar->Box(pt_search, 100);
    *cloudFinal += *boxEdge;
    *cloudFinal += *boxPlanar;
    publish_cloud(timeStartTime, cloudFinal, pubCloudBox);
    time_point_last_publish_box_ = Clock::now();
  }

  //  if (
  //    TimeKeeperSequential::millis_passed_since(time_point_last_publish_all_) > 10000 &&
  //    publish_things_)
  {
    publish_cloud(timeStartTime, occ_all_ds->cloud, pubCloudAll);
    time_point_last_publish_all_ = Clock::now();
  }


  //    *cloudFullAll += *cloudFullTrans;


  //    publish_cloud(timeStart, cloudFullTrans, pubCloudFullLast);

  if (publish_things_) {
    //    geometry_msgs::msg::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
    //      transformEstimatedLast[2], -transformEstimatedLast[0], -transformEstimatedLast[1]);

    tf2::Quaternion geoQuat;
    geoQuat.setRPY(transformEstimatedLast[0], transformEstimatedLast[1], transformEstimatedLast[2]);

    nav_msgs::msg::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "torch";

    odomAftMapped.header.stamp = mat_odom->header_in->stamp;
    odomAftMapped.pose.pose.orientation.x = geoQuat.x();
    odomAftMapped.pose.pose.orientation.y = geoQuat.y();
    odomAftMapped.pose.pose.orientation.z = geoQuat.z();
    odomAftMapped.pose.pose.orientation.w = geoQuat.w();
    odomAftMapped.pose.pose.position.x = transformEstimatedLast[3];
    odomAftMapped.pose.pose.position.y = transformEstimatedLast[4];
    odomAftMapped.pose.pose.position.z = transformEstimatedLast[5];
    odomAftMapped.twist.twist.angular.x = transformInLast[0];
    odomAftMapped.twist.twist.angular.y = transformInLast[1];
    odomAftMapped.twist.twist.angular.z = transformInLast[2];
    odomAftMapped.twist.twist.linear.x = transformInLast[3];
    odomAftMapped.twist.twist.linear.y = transformInLast[4];
    odomAftMapped.twist.twist.linear.z = transformInLast[5];
    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped aftMappedTrans;
    aftMappedTrans.header.frame_id = "map";
    aftMappedTrans.child_frame_id = "torch";
    aftMappedTrans.header.stamp = timeStartTime;
    aftMappedTrans.transform.rotation = odomAftMapped.pose.pose.orientation;
    aftMappedTrans.transform.translation.set__x(odomAftMapped.pose.pose.position.x);
    aftMappedTrans.transform.translation.set__y(odomAftMapped.pose.pose.position.y);
    aftMappedTrans.transform.translation.set__z(odomAftMapped.pose.pose.position.z);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster(node_);
    static_broadcaster.sendTransform(aftMappedTrans);
  }

  time_keeper.add_time_point("End");
//  time_keeper.print_times();
  counter++;
  //    *cloudDsEdge += *cloudDsPlanar;


  //  nav_msgs::msg::Odometry odom_aft_map;
  //  {
  //    tf2::Quaternion quat;
  //    quat.setRPY(transformEstimatedLast[2], -transformEstimatedLast[0],
  //    -transformEstimatedLast[1]); odom_aft_map.header.stamp = timeStartTime;
  //    odom_aft_map.pose.pose.orientation.x = quat.x();
  //    odom_aft_map.pose.pose.orientation.y = quat.y();
  //    odom_aft_map.pose.pose.orientation.z = quat.z();
  //    odom_aft_map.pose.pose.orientation.w = quat.w();
  //    odom_aft_map.pose.pose.position.x = transformEstimatedLast(3);
  //    odom_aft_map.pose.pose.position.y = transformEstimatedLast(4);
  //    odom_aft_map.pose.pose.position.z = transformEstimatedLast(5);
  //  }
  geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr_slam =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  {
    tf2::Quaternion quat;
    quat.setRPY(transformEstimatedLast[2], -transformEstimatedLast[0], -transformEstimatedLast[1]);
    transform_stamped_ptr_slam->header.stamp = timeStartTime;
    transform_stamped_ptr_slam->transform.rotation.x = quat.x();
    transform_stamped_ptr_slam->transform.rotation.y = quat.y();
    transform_stamped_ptr_slam->transform.rotation.z = quat.z();
    transform_stamped_ptr_slam->transform.rotation.w = quat.w();
    transform_stamped_ptr_slam->transform.translation.x = transformEstimatedLast(3);
    transform_stamped_ptr_slam->transform.translation.y = transformEstimatedLast(4);
    transform_stamped_ptr_slam->transform.translation.z = transformEstimatedLast(5);
    transform_stamped_ptr_slam->child_frame_id = "lidar_slam";
    transform_stamped_ptr_slam->header.frame_id = "slam_origin";
  }
  callback_slam_transform_(transform_stamped_ptr_slam);


  //  ProConMaterialMother::Ptr mat_dummy = std::make_shared<ProConMaterialMother>();
  return mat_dummy;
}

void MapperOcc::publish_cloud(
  rclcpp::Time & time,
  Cloud::Ptr cloud_in,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher)
{
  sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud =
    PclUtils::cloud_pcl_to_pointcloud2(cloud_in, "map");
  msg_cloud->header.stamp = time;
  publisher->publish(*msg_cloud);
}

void MapperOcc::concatenateTransformation(
  const Eigen::VectorXd & transformIn,
  const Eigen::VectorXd & transformBefMapped,
  const Eigen::VectorXd & transformAftMapped,
  Eigen::VectorXd & transformTobeMapped)
{
  transformTobeMapped.setZero(6);

  Eigen::Matrix3d R1, R2, R3, RR;
  R1 = make_mat_rot(transformIn, "ZXY");
  R2 = make_mat_rot(transformBefMapped, "ZXY");
  R3 = make_mat_rot(transformAftMapped, "ZXY");
  Eigen::Vector3d t, tt;
  t = transformBefMapped.tail(3) - transformIn.tail(3);
  RR = R1 * R2.transpose() * R3;
  tt = RR.transpose() * R1 * t;
  Eigen::VectorXd corrected_Odom(6);
  transformTobeMapped(0) = -asin(RR(2, 1));
  transformTobeMapped(1) =
    atan2(RR(2, 0) / cos(transformTobeMapped(0)), RR(2, 2) / cos(transformTobeMapped(0)));
  transformTobeMapped(2) =
    atan2(RR(0, 1) / cos(transformTobeMapped(0)), RR(1, 1) / cos(transformTobeMapped(0)));
  transformTobeMapped.tail(3) = transformAftMapped.tail(3) - tt;
}

void MapperOcc::pointAssociateToMap(
  const Point & pi, Point & po, const Eigen::VectorXd & transformTobeMapped)
{
  Eigen::Matrix3d R = make_mat_rot(transformTobeMapped, "ZXY");
  Eigen::Vector3d t, tt;
  t << pi.x, pi.y, pi.z;
  tt = (R.transpose() * t) + transformTobeMapped.tail(3);
  po.x = static_cast<float>(tt(0));
  po.y = static_cast<float>(tt(1));
  po.z = static_cast<float>(tt(2));
  po.intensity = pi.intensity;
}

float MapperOcc::random_color(uint64_t seed, float min, float max)
{
  std::mt19937 rng;
  rng.seed(seed);
  std::uniform_real_distribution<> dist6(min, max);
  return static_cast<float>(dist6(rng));
}

Cloud::Ptr MapperOcc::TransformCloud(
  const Cloud::ConstPtr cloudIn, const Eigen::VectorXd & transform6)
{
  Cloud::Ptr cloudOut(new Cloud());
  cloudOut->points.resize(cloudIn->size());

  std::transform(
    std::execution::par,
    cloudIn->points.begin(),
    cloudIn->points.end(),
    cloudOut->points.begin(),
    [this, transform6](const Point & point) {
      Point point_trans;
      pointAssociateToMap(point, point_trans, transform6);
      return point_trans;
    });

  return cloudOut;
}

std::vector<Eigen::VectorXd> MapperOcc::LevenbergAsync(
  const Cloud::Ptr & cloudDsFeat,
  const Cloud::Ptr & cloudBoxFeat,
  const Kdtree::Ptr & kdTreeBoxFeat,
  int numKNeighbours,
  const TransEstimator & transEstimator,
  bool IsEdge) const
{
  const size_t points_size = cloudDsFeat->points.size();

  std::vector<Eigen::VectorXd> a_lines(points_size);
  std::vector<std::bitset<1>> should_use_a_lines(points_size);

  std::transform(
    std::execution::par,
    cloudDsFeat->points.begin(),
    cloudDsFeat->points.end(),
    tbb::make_zip_iterator(a_lines.begin(), should_use_a_lines.begin()),
    [&kdTreeBoxFeat, &transEstimator, numKNeighbours, &cloudBoxFeat, IsEdge](const Point & point) {
      Point pointSel;
      Eigen::Vector3d newPoint = transEstimator.pointAssociateToMap(point);
      pointSel.x = static_cast<float>(newPoint(0));
      pointSel.y = static_cast<float>(newPoint(1));
      pointSel.z = static_cast<float>(newPoint(2));
      std::vector<int> pointSearchInd;
      std::vector<float> pointSearchSqDis;
      kdTreeBoxFeat->nearestKSearch(pointSel, numKNeighbours, pointSearchInd, pointSearchSqDis);
      Eigen::VectorXd Aline(7);
      if (pointSearchSqDis.back() < 1.0) {
        Cloud::Ptr neighbours(new Cloud());
        for (const auto & ind : pointSearchInd) {
          neighbours->points.push_back(cloudBoxFeat->points[ind]);
        }
        if (transEstimator.coeffsCalculator(point, neighbours, IsEdge, Aline)) {
          return std::make_tuple(Aline, std::bitset<1>{true});
        }
      }

      return std::make_tuple(Aline, std::bitset<1>{false});
    });

  std::vector<Eigen::VectorXd> a_lines_final;

  std::for_each(
    std::execution::seq,
    tbb::make_zip_iterator(a_lines.begin(), should_use_a_lines.begin()),
    tbb::make_zip_iterator(a_lines.end(), should_use_a_lines.end()),
    [&a_lines_final](const std::tuple<Eigen::VectorXd, std::bitset<1>> & aline_and_should_use) {
      Eigen::VectorXd a_line = std::get<0>(aline_and_should_use);
      bool should_use = std::get<1>(aline_and_should_use)[0];
      if (should_use) {a_lines_final.push_back(a_line);}
    });

  return a_lines_final;
}

ProducerConsumerMaster::FuncType MapperOcc::GetMethod()
{
  using std::placeholders::_1;
  return std::bind(&MapperOcc::Process, this, _1);
}

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora
