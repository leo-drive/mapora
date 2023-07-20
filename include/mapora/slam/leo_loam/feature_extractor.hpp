// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__SLAM__LEO_LOAM__FEATURE_EXTRACTOR_HPP_
#define MAPORA__SLAM__LEO_LOAM__FEATURE_EXTRACTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mapora/point_xyzi.hpp>

#include <memory>
#include <vector>

#include "mapora/pcl_utils.hpp"

#include "mapora/slam/leo_loam/queue_management/material_01_points_raw.hpp"
#include "mapora/slam/leo_loam/queue_management/material_02_features.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class FeatureExtractor
{
public:
  using SharedPtr = std::shared_ptr<FeatureExtractor>;

  using Point = PclUtils::PointPcl;
  using Cloud = PclUtils::CloudPcl;

  explicit FeatureExtractor(rclcpp::Node & node);

  ProducerConsumerMaster::FuncType GetMethod();

private:
  struct FeatureExtractorResult
  {
    Cloud::ConstPtr cloud_full;

    Cloud::Ptr cloud_occluded_right;
    Cloud::Ptr cloud_occluded_left;
    Cloud::Ptr cloud_lonely;

    Cloud::Ptr cloud_edge;
    Cloud::Ptr cloud_edge_pot;
    Cloud::Ptr cloud_planar;
    Cloud::Ptr cloud_planar_pot;

    Cloud::Ptr cloud_discard1;
    Cloud::Ptr cloud_discard_neighbors_edge;
    Cloud::Ptr cloud_discard3;
    Cloud::Ptr cloud_discard_neighbors_planar;
    Cloud::Ptr cloud_discard_angle_disparity;

    std::vector<double> durations;

    FeatureExtractorResult()
    {
      cloud_occluded_right.reset(new Cloud);
      cloud_occluded_left.reset(new Cloud);
      cloud_lonely.reset(new Cloud);

      cloud_edge.reset(new Cloud);
      cloud_edge_pot.reset(new Cloud);
      cloud_planar.reset(new Cloud);
      cloud_planar_pot.reset(new Cloud);

      cloud_discard1.reset(new Cloud);
      cloud_discard_neighbors_edge.reset(new Cloud);
      cloud_discard3.reset(new Cloud);
      cloud_discard_neighbors_planar.reset(new Cloud);
      cloud_discard_angle_disparity.reset(new Cloud);

      durations = std::vector<double>(10, 0.0);
    }

    FeatureExtractorResult & operator+=(const FeatureExtractorResult & other)
    {
      * cloud_occluded_right += *other.cloud_occluded_right;
      * cloud_occluded_left += *other.cloud_occluded_left;
      * cloud_lonely += *other.cloud_lonely;
      * cloud_edge += *other.cloud_edge;
      * cloud_edge_pot += *other.cloud_edge_pot;
      * cloud_planar += *other.cloud_planar;
      * cloud_planar_pot += *other.cloud_planar_pot;
      * cloud_discard1 += *other.cloud_discard1;
      * cloud_discard_neighbors_edge += *other.cloud_discard_neighbors_edge;
      * cloud_discard3 += *other.cloud_discard3;
      * cloud_discard_neighbors_planar += *other.cloud_discard_neighbors_planar;
      * cloud_discard_angle_disparity += *other.cloud_discard_angle_disparity;

      assert(durations.size() == other.durations.size());
      for (size_t kI = 0; kI < durations.size(); ++kI) {
        durations.at(kI) += other.durations.at(kI);
      }

      return *this;
    }
  };

  std::vector<float> angles_vertical_{
    -25.0f, -19.582f, -16.042f, -13.565f, -11.742f, -10.346f, -9.244f, -8.352f, -7.65f, -7.15f,
    -6.85f, -6.65f, -6.5f, -6.39f, -6.28f, -6.17f, -6.06f, -5.95f, -5.84f, -5.73f,
    -5.62f, -5.51f, -5.4f, -5.29f, -5.18f, -5.07f, -4.96f, -4.85f, -4.74f, -4.63f,
    -4.52f, -4.41f, -4.3f, -4.19f, -4.08f, -3.97f, -3.86f, -3.75f, -3.64f, -3.53f,
    -3.42f, -3.31f, -3.2f, -3.09f, -2.98f, -2.87f, -2.76f, -2.65f, -2.54f, -2.43f,
    -2.32f, -2.21f, -2.1f, -1.99f, -1.88f, -1.77f, -1.66f, -1.55f, -1.44f, -1.33f,
    -1.22f, -1.11f, -1.0f, -0.89f, -0.78f, -0.67f, -0.56f, -0.45f, -0.34f, -0.23f,
    -0.12f, -0.01f, 0.1f, 0.21f, 0.32f, 0.43f, 0.54f, 0.65f, 0.76f, 0.87f,
    0.98f, 1.09f, 1.2f, 1.31f, 1.42f, 1.53f, 1.64f, 1.75f, 1.86f, 1.97f,
    2.08f, 2.19f, 2.3f, 2.41f, 2.52f, 2.63f, 2.74f, 2.85f, 2.96f, 3.07f,
    3.18f, 3.29f, 3.4f, 3.51f, 3.62f, 3.73f, 3.84f, 3.95f, 4.06f, 4.17f,
    4.28f, 4.39f, 4.5f, 4.61f, 4.72f, 4.83f, 4.98f, 5.18f, 5.43f, 5.73f,
    6.08f, 6.48f, 6.98f, 7.58f, 8.43f, 9.7f, 11.75f, 15.0f};

  int count_total_run_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_ringy;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_occluded_right;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_occluded_left;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_lonely;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_edge;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_edge_pot;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_planar;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_planar_pot;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_discard1;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_discard_neighbors_edge;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_discard3;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_discard_neighbors_planar;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_discard_angle_disparity_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_tips;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_rainbow_spiral;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_in_rainbow_;


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_raw_;

  template<typename BidirectionalIterator, typename T>
  BidirectionalIterator getClosest(
    BidirectionalIterator first, BidirectionalIterator last, const T & value)
  {
    BidirectionalIterator before = std::lower_bound(first, last, value);

    if (before == first) {return first;}
    if (before == last) {
      return --last;  // iterator must be bidirectional
    }
    BidirectionalIterator after = before;
    --before;

    return (*after - value) < (value - *before) ? after : before;
  }

  template<typename BidirectionalIterator, typename T>
  std::size_t getClosestIndex(
    BidirectionalIterator first, BidirectionalIterator last, const T & value)
  {
    return std::distance(first, getClosest(first, last, value));
  }

  std::vector<Cloud::Ptr> CloudRawToRings(const Cloud::ConstPtr & cloud_in);

  FeatureExtractorResult ExtractFeatures(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg_cloud);

  ProConMaterialBase::Ptr Process(ProConMaterialBase::Ptr mat_in);
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora


#endif  // MAPORA__SLAM__LEO_LOAM__FEATURE_EXTRACTOR_HPP_
