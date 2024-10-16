/*
* Copyright 2023 LeoDrive.ai, Inc. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
 */

#ifndef MAPORA__MAPORA_HPP_
#define MAPORA__MAPORA_HPP_

#include "mapora/points_provider.hpp"
#include "mapora/transform_provider.hpp"
#include "mapora/visibility_control.hpp"
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <future>
#include <mutex>

namespace mapora
{
class Mapora : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<Mapora>;
  using ConstSharedPtr = const std::shared_ptr<Mapora>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Point = point_types::PointXYZITRH;
  using Points = std::vector<Point>;

  explicit Mapora(const rclcpp::NodeOptions & options);

  void process();

  // params
  double imu2lidar_roll;
  double imu2lidar_pitch;
  double imu2lidar_yaw;
  std::string pcap_dir_path_;
  std::string pose_txt_path_;
  std::string project_namespace_;
  double max_point_distance_from_lidar_;
  double min_point_distance_from_lidar_;
  std::string las_export_dir;
  std::string lidar_model;

  std::vector<Points> clouds;

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_cloud_current_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ptr_path;

  transform_provider::TransformProvider::SharedPtr
      transform_provider;
  points_provider::PointsProvider::SharedPtr points_provider_;

  PointCloud2::SharedPtr points_to_cloud(const Points & points_bad, const std::string & frame_id);

  void callback_cloud_surround_out(const Points & points_surround);

  Points transform_points(Points & cloud);


  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  transform_provider::TransformProvider::Pose pose;

  Point last_point;

  int file_counter = 0;
};
}  // namespace mapora

#endif  // MAPORA__MAPORA_HPP_
