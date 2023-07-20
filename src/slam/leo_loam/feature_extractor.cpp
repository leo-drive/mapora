// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/slam/leo_loam/feature_extractor.hpp"

#include <tbb/iterators.h>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <execution>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <memory>
#include <string>
#include <tuple>


namespace mapora
{
namespace slam
{
namespace leo_loam
{
using Point = FeatureExtractor::Point;
using Cloud = FeatureExtractor::Cloud;

FeatureExtractor::FeatureExtractor(rclcpp::Node & node)
: count_total_run_(0),
  pub_cloud_ringy{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_ringy", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_occluded_right{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_occluded_right", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_occluded_left{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_occluded_left", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_lonely{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_lonely", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_edge{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_edge", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_edge_pot{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_edge_pot", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_planar{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_planar", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_planar_pot{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_planar_pot", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_discard1{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_discard1", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_discard_neighbors_edge{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_discard_neighbors_edge", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_discard3{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_discard3", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_discard_neighbors_planar{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_discard_neighbors_planar", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_discard_angle_disparity_{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_discard_angle_disparity", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_tips{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_tips", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_rainbow_spiral{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_rainbow_spiral", rclcpp::QoS(rclcpp::KeepLast(1)))},
  pub_cloud_in_rainbow_{node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_in_rainbow", rclcpp::QoS(rclcpp::KeepLast(1)))}
{
}

std::vector<Cloud::Ptr> FeatureExtractor::CloudRawToRings(const Cloud::ConstPtr & cloud_in)
{
  const int laser_count = angles_vertical_.size();
  const int count_points = cloud_in->points.size();

  std::vector<Cloud::Ptr> rings(laser_count);
  for (auto & ring : rings) {
    ring.reset(new Cloud);
  }

  std::vector<int> vec_ring_ids(count_points);
  std::vector<float> vec_ringified(count_points);

  const auto & point_cloud_in_first = cloud_in->points.front();
  const auto & point_cloud_in_last = cloud_in->points.back();
  float angle_horizontal_point_first = -atan2(point_cloud_in_first.y, point_cloud_in_first.x);
  float angle_horizontal_point_last =
    -atan2(point_cloud_in_last.y, point_cloud_in_last.x) + 2 * M_PI;

  if (angle_horizontal_point_last - angle_horizontal_point_first > 3 * M_PI) {
    angle_horizontal_point_last -= 2 * M_PI;
  } else if (angle_horizontal_point_last - angle_horizontal_point_first < M_PI) {
    angle_horizontal_point_last += 2 * M_PI;
  }

  std::transform(
    std::execution::par,
    cloud_in->points.begin(),
    cloud_in->points.end(),
    tbb::make_zip_iterator(vec_ringified.begin(), vec_ring_ids.begin()),
    [this, angle_horizontal_point_first, angle_horizontal_point_last](
      const Point & p) -> std::tuple<float, int> {
      double dist_xy = sqrt(pow(p.x, 2) + pow(p.y, 2));
      float angle_vertical = atan2(p.z, dist_xy) / M_PI * 180.0;

      int ring =
      getClosestIndex(angles_vertical_.cbegin(), angles_vertical_.cend(), angle_vertical);
      int laser_count = angles_vertical_.size();
      if (ring < laser_count / 2) {
        ring = ring * 2;
      } else {
        ring = (ring - laser_count / 2) * 2 + 1;
      }

      float angle_horizontal_point = -atan2(p.y, p.x);
      if (angle_horizontal_point - angle_horizontal_point_first <= M_PI) {
        if (angle_horizontal_point < angle_horizontal_point_first - M_PI / 2) {
          angle_horizontal_point += 2 * M_PI;
        } else if (angle_horizontal_point > angle_horizontal_point_first + M_PI * 3 / 2) {
          angle_horizontal_point -= 2 * M_PI;
        }
      } else {
        angle_horizontal_point += 2 * M_PI;

        if (angle_horizontal_point < angle_horizontal_point_last - M_PI * 3 / 2) {
          angle_horizontal_point += 2 * M_PI;
        } else if (angle_horizontal_point > angle_horizontal_point_last + M_PI / 2) {
          angle_horizontal_point -= 2 * M_PI;
        }
      }

      float angle_offset_from_first = angle_horizontal_point - angle_horizontal_point_first;
      float angle_dif = angle_horizontal_point_last - angle_horizontal_point_first;
      float ratio_clockwise = angle_offset_from_first / angle_dif;

      return std::make_tuple(static_cast<float>(ring + 0.1 * ratio_clockwise), ring);
    });

  typedef std::tuple<Point, float, int> WeirdType;

//  std::cout << "vec_ringified: " << vec_ringified.size() << std::endl;
//  std::cout << "vec_ring_ids: " << vec_ring_ids.size() << std::endl;
//  std::cout << "cloud_in->points: " << cloud_in->points.size() << std::endl;
  std::for_each(
    std::execution::seq,
    tbb::make_zip_iterator(cloud_in->points.begin(), vec_ringified.begin(), vec_ring_ids.begin()),
    tbb::make_zip_iterator(cloud_in->points.end(), vec_ringified.end(), vec_ring_ids.end()),
    [&rings](const WeirdType & tuppy) {
      const Point & p = std::get<0>(tuppy);
      const float & ringy = std::get<1>(tuppy);
      const int & ring = std::get<2>(tuppy);
      Point p2 = p;
      p2.intensity = ringy;
      rings.at(ring)->points.push_back(p2);
    });

  return rings;
}

FeatureExtractor::FeatureExtractorResult FeatureExtractor::ExtractFeatures(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg_cloud)
{
  count_total_run_++;

  Cloud::Ptr cloud_in(new Cloud);
  cloud_in = PclUtils::pointcloud2_to_cloud_pcl(msg_cloud);
  const auto & header_in = msg_cloud->header;
  const auto & time_in = header_in.stamp;
  const auto & frame_in = header_in.frame_id;

  if (cloud_in->points.size() < 500) {throw std::logic_error("Not enough points in cloud.");}

  int laser_count = angles_vertical_.size();

  {
    using Point = point_types::PointXYZI;
    using CloudView = point_cloud_msg_wrapper::PointCloud2View<Point>;
    CloudView cloud_view(*msg_cloud);

    using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<Point>;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_ptr =
      std::make_shared<sensor_msgs::msg::PointCloud2>();
    CloudModifier cloud_modifier_in_rainbow(*cloud_ptr, "map");
    const size_t count_points = cloud_view.size();
    cloud_modifier_in_rainbow.resize(count_points);

    for (size_t i = 0; i < count_points; ++i) {
      cloud_modifier_in_rainbow[i] = cloud_view[i];
      cloud_modifier_in_rainbow[i].intensity = i * 0.05f;
    }

    pub_cloud_in_rainbow_->publish(*cloud_ptr);
  }

  Cloud::Ptr cloud_ringy(new Cloud());

  std::vector<Cloud::Ptr> rings(laser_count);
  for (auto & ring : rings) {
    ring.reset(new Cloud);
  }

  rings = CloudRawToRings(cloud_in);

  for (const auto & ring : rings) {
    *cloud_ringy += *ring;
  }

  // Process each ring

  typedef std::tuple<Cloud::ConstPtr, int> RingIndTuple;

  struct Confingy
  {
    double RemovalFactorOcclusion = 0.1;
    double RemovalFactorDepthDifference = 0.0002;
    double DiscardThresholdAngleDisparity = 10;

    int SubRegionCount = 6;
    int ClassificationSearchPointCount = 5;

    double ClassificationThresholdEdge = 1.0;
    double ClassificationThresholdPlanar = 0.1;

    int ClassificationThresholdPointCountEdge = 4;
    int ClassificationThresholdPointCountEdgeWorse = 20;

    int ClassificationThresholdPointCountPlanar = 4;

    double ClassificationThresholdDistanceIgnoreNeighbor = 0.05;
    double ClassificationPlanarWorseDownsampleLeafSize = 0.5;

    //    double IgnoreUpToYPositive = 2.0;
    //    double IgnoreUpToYNegative = -1.5;
    //    double IgnoreUpToXPositive = 1.0;
    //    double IgnoreUpToXNegative = -1.0;

    double IgnoreUpToYPositive = 1.4;
    double IgnoreUpToYNegative = -1.4;
    double IgnoreUpToXPositive = 3.5;
    double IgnoreUpToXNegative = -1.7;
  };

  Confingy config;

  auto ProcessRing = [&config](const RingIndTuple & ring_and_ind) {
      Cloud::ConstPtr ring_current = std::get<0>(ring_and_ind);
      //    int ind_ring = std::get<1>(ring_and_ind);

      FeatureExtractorResult result;
      int radius_search = config.ClassificationSearchPointCount;
      int subregions = config.SubRegionCount;


      // Calculate curvatures
      //    ROS_INFO_STREAM("Current Ring: " << ind_ring << " valid: "
      //    << (ring_current != nullptr));
      //    ROS_INFO_STREAM("Current Ring: " << ind_ring << " count: " <<
      //    ring_current->points.size());

      const int count_points = ring_current->points.size();

      if (count_points < (subregions + 2) * radius_search * 2) {return result;}

      std::vector<float> cloudCurvature(count_points);
      std::vector<int> cloudSortInd(count_points);
      std::vector<bool> cloud_discarded(count_points);
      std::vector<int> cloud_label(count_points);

      for (int i = radius_search; i < count_points - radius_search; i++) {
        const auto & points = ring_current->points;
        double diffX = 0;
        double diffY = 0;
        double diffZ = 0;
        const auto & point_mid = points.at(i);
        for (int j = -radius_search; j < radius_search + 1; ++j) {
          if (j == 0) {continue;}
          const auto & point = points.at(i + j);
          diffX += point.x - point_mid.x;
          diffY += point.y - point_mid.y;
          diffZ += point.z - point_mid.z;
        }

        cloudCurvature.at(i) = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd.at(i) = i;
        cloud_discarded.at(i) = false;
        cloud_label.at(i) = 0;
      }

      // Mark discarded points

      // Ignore points with high azimuth discrepancy or within the rectangle boundaries
      double azim_dif_max_expected = config.DiscardThresholdAngleDisparity;
      // deg
      for (int ind_point = radius_search; ind_point < count_points - radius_search; ++ind_point) {
        if (cloud_discarded.at(ind_point)) {continue;}
        const auto & point_curr = ring_current->points.at(ind_point);

        if (
          point_curr.x < config.IgnoreUpToXPositive && point_curr.x > config.IgnoreUpToXNegative &&
          point_curr.y < config.IgnoreUpToYPositive && point_curr.y > config.IgnoreUpToYNegative)
        {
          cloud_discarded.at(ind_point) = true;
          result.cloud_lonely->points.push_back(point_curr);
        }

        const auto & point_prev = ring_current->points.at(ind_point - 1);

        double azim_curr = -atan2(point_curr.y, point_curr.x) * 180.0 / M_PI;
        double azim_prev = -atan2(point_prev.y, point_prev.x) * 180.0 / M_PI;
        double azim_diff = std::abs(azim_curr - azim_prev);

        //      std::cout << "ind: " << ind_point << std::endl;
        //      std::cout << "dif: " << std::abs(azim_curr - azim_prev) << std::endl;
        //      std::cout << "azim_curr: " << azim_curr << std::endl;
        //      std::cout << "azim_dif_max_expected: " << azim_dif_max_expected << std::endl;

        if (
          azim_diff > azim_dif_max_expected && (azim_diff < 360.0 - azim_dif_max_expected / 2.0 ||
          azim_diff > 360.0 + azim_dif_max_expected / 2.0))
        {
          // discard from previous  - radius_search to +radius_search
          for (int kI = ind_point - radius_search; kI < ind_point + radius_search + 1; ++kI) {
            cloud_discarded.at(kI) = true;

            const auto & point_to_be_discarded = ring_current->points.at(kI);
            result.cloud_discard_angle_disparity->points.push_back(point_to_be_discarded);
            //          std::cout << "DISCARDED" << std::endl;
          }
        }
      }

      for (size_t i = radius_search; i < ring_current->points.size() - 1 - radius_search; i++) {
        // Discard occluded points
        const Point & point_prev = ring_current->points.at(i - 1);
        const Point & point_curr = ring_current->points.at(i);
        const Point & point_next = ring_current->points.at(i + 1);

        double dist_cur_to_next = PclUtils::distance(point_curr, point_next);

        double removal_factor_occlusion = config.RemovalFactorOcclusion;
        if (dist_cur_to_next > std::sqrt(removal_factor_occlusion)) {
          double depth1 = PclUtils::norm(point_curr);
          double depth2 = PclUtils::norm(point_next);

          if (depth1 > depth2) {
            // Occluded Left
            double diffX = point_next.x - point_curr.x * depth2 / depth1;
            double diffY = point_next.y - point_curr.y * depth2 / depth1;
            double diffZ = point_next.z - point_curr.z * depth2 / depth1;

            if (
              std::sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 <
              removal_factor_occlusion)
            {
              for (size_t j = i - radius_search; j < i + 1; ++j) {
                cloud_discarded.at(j) = true;
                const Point & point_bad_neighbor = ring_current->points.at(j);
                result.cloud_occluded_left->points.push_back(point_bad_neighbor);
              }
            }
          } else {
            // Occluded Right
            double diffX = point_next.x * depth1 / depth2 - point_curr.x;
            double diffY = point_next.y * depth1 / depth2 - point_curr.y;
            double diffZ = point_next.z * depth1 / depth2 - point_curr.z;

            if (
              std::sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 <
              removal_factor_occlusion)
            {
              for (size_t j = i + 1; j < i + radius_search + 2; ++j) {
                cloud_discarded.at(j) = true;
                const Point & point_bad_neighbor = ring_current->points.at(j);
                result.cloud_occluded_right->points.push_back(point_bad_neighbor);
              }
            }
          }
        }

        // Too far away from his neighbors compared to its distance to lidar.
        double dist_cur_to_prev = PclUtils::distance(point_curr, point_prev);

        double dist_cur = PclUtils::norm(point_curr);

        double removal_factor_depth_difference = config.RemovalFactorDepthDifference;

        if (
          dist_cur_to_next > std::sqrt(removal_factor_depth_difference) * dist_cur &&
          dist_cur_to_prev > std::sqrt(removal_factor_depth_difference) * dist_cur)
        {
          cloud_discarded.at(i) = true;
          result.cloud_lonely->points.push_back(point_curr);
        }
      }

      // Sort them by their curvatures
      // Extract features and add them to their respective point clouds


      // For each ring
      if (count_points < subregions) {return result;}
      Cloud::Ptr cloud_planar_pot_raw(new Cloud());

      //    std::cout << "laser i: " << i << std::endl;
      //    std::cout << "scanStartInd i: " << scanStartInd.at(i) << std::endl;
      //    std::cout << "scanEndInd i: " << scanEndInd.at(i) << std::endl;
      //    std::cout << "ring: " << ind_ring << std::endl;

      for (int j = 0; j < subregions; j++) {
        int sp = count_points * j / subregions + radius_search;
        int ep = count_points * (j + 1) / subregions - 1 - radius_search;

        //      std::cout << "count_points: " << count_points << std::endl;
        //      std::cout << "subregion: " << j << ", sp: " << sp << ", ep: " << ep << std::endl;


        // Sort each subregion current ring i based on their curvatures
        // Sort from small to big
        auto comp = [&cloudCurvature](int a, int b) {
            return cloudCurvature.at(a) < cloudCurvature.at(b);
          };
        std::sort(cloudSortInd.begin() + sp, cloudSortInd.begin() + ep, comp);


        // Look For Edge/EdgePot Points
        int ind_sorted_largest = 0;
        // Iterate reversed for high to low curvatured indices
        for (int k = ep; k >= sp; k--) {
          int ind = cloudSortInd.at(k);
          // Will Choose Edge Points
          // Edge Points have high curvatures
          // Skip if discarded or curvature is low
          if (
            cloud_discarded.at(ind) == true ||
            cloudCurvature.at(ind) <= config.ClassificationThresholdEdge)
          {
            continue;
          }
          auto & point_cur = ring_current->points.at(ind);

          ind_sorted_largest++;
          if (ind_sorted_largest <= config.ClassificationThresholdPointCountEdge) {
            // It's in top 2 in its region
            // Select it as Edge and EdgePot point
            cloud_label.at(ind) = 2;
            result.cloud_edge->push_back(point_cur);
            result.cloud_edge_pot->push_back(point_cur);
          } else if (ind_sorted_largest <= config.ClassificationThresholdPointCountEdgeWorse) {
            // It's in top 20 in its region
            // Select it as EdgePot point
            cloud_label.at(ind) = 1;
            result.cloud_edge_pot->push_back(point_cur);
          } else {
            // This point isn't qualified enough to be an Edge Point
            // We will break because we won't kill its neighbors
            break;
          }

          // Since this is an Edge/EdgePot Point, kill its neighbors


          // Discard Edge/EdgePot itself
          cloud_discarded.at(ind) = true;
          result.cloud_discard1->points.push_back(ring_current->points.at(ind));

          // Kill neighbors of Edge/EdgePot in positive direction
          for (int l = 1; l <= radius_search; l++) {
            //          std::cout << "positive killing: ind + l=" << ind + l << ", ind + l - 1=" <<
            //          ind + l - 1 << std::endl;
            if (
              PclUtils::distance(
                ring_current->points.at(ind + l), ring_current->points.at(ind + l - 1)) >
              std::sqrt(config.ClassificationThresholdDistanceIgnoreNeighbor))
            {
              // It's too far, he is not involved with his neighbor
              // Stop Neighbor killing in this direction
              break;
            }

            // It's involved with the Edge Point, kill it
            cloud_discarded.at(ind + l) = true;
            result.cloud_discard_neighbors_edge->points.push_back(ring_current->points.at(ind + l));
          }

          // Kill neighbors of Edge/EdgePot in negative direction
          for (int l = -1; l >= -radius_search; l--) {
            //          std::cout << "negative killing: ind + l=" << ind + l << ", ind + l + 1=" <<
            //          ind + l + 1 << std::endl;
            if (
              PclUtils::distance(
                ring_current->points.at(ind + l), ring_current->points.at(ind + l + 1)) >
              std::sqrt(config.ClassificationThresholdDistanceIgnoreNeighbor))
            {
              // It's too far, he is not involved with his neighbor
              // Stop Neighbor killing in this direction
              break;
            }

            // It's involved with the Edge Point, kill it
            cloud_discarded.at(ind + l) = true;
            result.cloud_discard_neighbors_edge->points.push_back(ring_current->points.at(ind + l));
          }
        }

        // Look For Planar/PlanarPot Points
        int ind_sorted_smallest = 0;
        // Iterate for low to high curvatured indices
        for (int k = sp; k <= ep; k++) {
          int ind = cloudSortInd.at(k);
          auto & point_cur = ring_current->points.at(ind);

          // Will Choose Planar Points
          // Planar Points have low curvatures
          // Skip if discarded or curvature is high
          if (
            cloud_discarded.at(ind) ||
            cloudCurvature.at(ind) >= config.ClassificationThresholdPlanar)
          {
            continue;
          }

          // Choose maximum 4 Planar points per region
          // It's in top 4 in its region as a successful Planar Point
          cloud_label.at(ind) = -1;
          result.cloud_planar->push_back(point_cur);

          ind_sorted_smallest++;
          if (ind_sorted_smallest >= config.ClassificationThresholdPointCountPlanar) {break;}

          cloud_discarded.at(ind) = true;
          result.cloud_discard3->points.push_back(point_cur);


          // Kill neighbors of Planar Point in positive direction
          for (int l = 1; l <= radius_search; l++) {
            if (
              PclUtils::distance(
                ring_current->points.at(ind + l), ring_current->points.at(ind + l - 1)) >
              std::sqrt(config.ClassificationThresholdDistanceIgnoreNeighbor))
            {
              // It's too far, he is not involved with his neighbor
              // Stop Neighbor killing in this direction
              break;
            }

            cloud_discarded.at(ind + l) = true;
            result.cloud_discard_neighbors_planar->points.push_back(
              ring_current->points.at(
                ind +
                l));
          }
          // Kill neighbors of Planar Point in negative direction
          for (int l = -1; l >= -radius_search; l--) {
            if (
              PclUtils::distance(
                ring_current->points.at(ind + l), ring_current->points.at(ind + l + 1)) >
              std::sqrt(config.ClassificationThresholdDistanceIgnoreNeighbor))
            {
              // It's too far, he is not involved with his neighbor
              // Stop Neighbor killing in this direction
              break;
            }

            cloud_discarded.at(ind + l) = true;
            result.cloud_discard_neighbors_planar->points.push_back(
              ring_current->points.at(
                ind +
                l));
          }
        }

        // Again, Iterate from low curvature to high
        for (int k = sp; k <= ep; k++) {
          // Every Point that is not Edge or EdgePot will be PlanarPot
          if (
            //          cloud_discarded.at(k) == 1 ||
            cloud_label.at(k) > 0)
          {
            continue;
          }
          cloud_planar_pot_raw->push_back(ring_current->points.at(k));
        }
      }

      Cloud::Ptr surfPointsLessFlatScanDS = PclUtils::downsample(
        cloud_planar_pot_raw, config.ClassificationPlanarWorseDownsampleLeafSize);
      *result.cloud_planar_pot += *surfPointsLessFlatScanDS;

      return result;
    };

  //  std::vector<std::future<FeatureExtractorResult>> future_extractors(rings.size());
  //
  //  for (int ind_ring = 0; ind_ring < rings.size(); ++ind_ring) {
  //    Cloud::ConstPtr ring_current = rings.at(ind_ring);
  //    future_extractors.at(ind_ring) = std::async(std::launch::async,
  //                                                ProcessRing,
  //                                                ring_current,
  //                                                ind_ring);
  //  }

  std::vector<int> vec_ring_ids(laser_count);
  std::iota(vec_ring_ids.begin(), vec_ring_ids.end(), 0);

  std::vector<FeatureExtractorResult> results(rings.size());
  std::transform(
    std::execution::par,
    tbb::make_zip_iterator(rings.begin(), vec_ring_ids.begin()),
    tbb::make_zip_iterator(rings.end(), vec_ring_ids.end()),
    results.begin(),
    ProcessRing);

  //  std::vector<FeatureExtractorResult> results(rings.size());
  //
  //  for (int ind_ring = 0; ind_ring < rings.size(); ++ind_ring) {
  //    results.at(ind_ring) = future_extractors.at(ind_ring).get();
  //  }

  FeatureExtractorResult results_combined;
  for (size_t ind_ring = 0; ind_ring < rings.size(); ++ind_ring) {
    results_combined += results.at(ind_ring);
  }

  Cloud::Ptr cloud_rainbow_spiral(new Cloud());
  //  for (int m = 0; m < cloud_in->points.size(); ++m) {
  //    Point point = cloud_in->points.at(m);
  //
  //    auto rgb = Helper::HsvToRgb(((float) m / cloud_in->points.size()) * 360,
  //                                0.6,
  //                                1);
  //    point.r = (uint8_t) rgb.r;
  //    point.g = (uint8_t) rgb.g;
  //    point.b = (uint8_t) rgb.b;
  //    cloud_rainbow_spiral->points.push_back(point);
  //  }

  Cloud::Ptr cloud_tips(new Cloud());

  cloud_tips->points.push_back(cloud_in->points.front());
  cloud_tips->points.push_back(cloud_in->points.back());

  const auto Publish = [](
    const Cloud::ConstPtr & cloud,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & pub,
    const rclcpp::Time & time,
    const std::string & frame) {
      sensor_msgs::msg::PointCloud2::SharedPtr msg =
        PclUtils::cloud_pcl_to_pointcloud2(cloud, frame);
      msg->header.set__stamp(time);
      pub.publish(*msg);
    };

  Publish(cloud_rainbow_spiral, *pub_cloud_rainbow_spiral, time_in, frame_in);
  Publish(cloud_ringy, *pub_cloud_ringy, time_in, frame_in);
  Publish(results_combined.cloud_edge, *pub_cloud_edge, time_in, frame_in);
  Publish(results_combined.cloud_edge_pot, *pub_cloud_edge_pot, time_in, frame_in);
  Publish(results_combined.cloud_planar, *pub_cloud_planar, time_in, frame_in);
  Publish(results_combined.cloud_planar_pot, *pub_cloud_planar_pot, time_in, frame_in);
  Publish(results_combined.cloud_occluded_right, *pub_cloud_occluded_right, time_in, frame_in);
  Publish(results_combined.cloud_occluded_left, *pub_cloud_occluded_left, time_in, frame_in);
  Publish(results_combined.cloud_lonely, *pub_cloud_lonely, time_in, frame_in);
  Publish(results_combined.cloud_discard1, *pub_cloud_discard1, time_in, frame_in);
  Publish(
    results_combined.cloud_discard_neighbors_edge,
    *pub_cloud_discard_neighbors_edge,
    time_in,
    frame_in);
  Publish(results_combined.cloud_discard3, *pub_cloud_discard3, time_in, frame_in);
  Publish(
    results_combined.cloud_discard_neighbors_planar,
    *pub_cloud_discard_neighbors_planar,
    time_in,
    frame_in);
  Publish(
    results_combined.cloud_discard_angle_disparity,
    *pub_cloud_discard_angle_disparity_,
    time_in,
    frame_in);
  Publish(cloud_tips, *pub_cloud_tips, time_in, frame_in);

  //  ROS_INFO_STREAM("t conversion: " << millis_passed_between(tt_01, tt_02));
  //  ROS_INFO_STREAM("t ringification: " << millis_passed_between(tt_02, tt_03));
  //  ROS_INFO_STREAM("t threads: " << millis_passed_between(tt_03, tt_04));
  //
  //  ROS_INFO_STREAM("t -- threads curvy took: " << results_combined.durations.at(0));
  //  ROS_INFO_STREAM("t -- threads discard took: " << results_combined.durations.at(1));
  //  ROS_INFO_STREAM("t -- threads sort took: " << results_combined.durations.at(2));
  //  ROS_INFO_STREAM("t -- threads classification took: " << results_combined.durations.at(3));
  //  ROS_INFO_STREAM("t -- threads downsampling took: " << results_combined.durations.at(4));
  //  ROS_INFO_STREAM("t -- threads all took: " << results_combined.durations.at(5));
  //
  //  ROS_INFO_STREAM("t combining: " << millis_passed_between(tt_04, tt_05));
  //  ROS_INFO_STREAM("t publishing: " << millis_passed_between(tt_05, tt_06));
  //  ROS_INFO_STREAM("t all: " << millis_passed_between(tt_01, tt_06));


    std::cout << "Count Total Run: " << count_total_run_ << std::endl;
  //  std::cout << "Size Edge: " << results_combined.cloud_edge->points.size() << std::endl;
  //  std::cout << "Size EdgePot: " << results_combined.cloud_edge_pot->points.size() << std::endl;
  //  std::cout << "Size Plan: " << results_combined.cloud_planar->points.size() << std::endl;
  //  std::cout << "Size PlanPot: " << results_combined.cloud_planar_pot->points.size() <<
  //  std::endl;

  results_combined.cloud_full = cloud_in;
//  std::cout << std::endl;
  return results_combined;
}

ProConMaterialBase::Ptr FeatureExtractor::Process(ProConMaterialBase::Ptr mat_in)
{
  auto mat_init = std::dynamic_pointer_cast<MaterialPointsRaw>(mat_in);
  if (!mat_init) {
    throw std::runtime_error("Cannot cast ProConMaterialMother to MaterialPointsRaw");
  }

  FeatureExtractorResult result = ExtractFeatures(mat_init->cloud_in);

  MaterialFeatures::Ptr mat_feat = std::make_shared<MaterialFeatures>();
  mat_feat->cloud_full = result.cloud_full;
  mat_feat->cloud_edge = result.cloud_edge;
  mat_feat->cloud_edge_pot = result.cloud_edge_pot;
  mat_feat->cloud_planar = result.cloud_planar;
  mat_feat->cloud_planar_pot = result.cloud_planar_pot;

  std_msgs::msg::Header::SharedPtr header(new std_msgs::msg::Header);
  *header = mat_init->cloud_in->header;

  mat_feat->header_in = header;
  //  mat_feat->use_imu = mat_init->use_imu;
  //  mat_feat->Roll = mat_init->Roll;
  //  mat_feat->Pitch = mat_init->Pitch;
  //  mat_feat->Yaw = mat_init->Yaw;
  return mat_feat;
}
ProducerConsumerMaster::FuncType FeatureExtractor::GetMethod()
{
  using std::placeholders::_1;
  return std::bind(&FeatureExtractor::Process, this, _1);
}

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora
