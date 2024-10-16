/*
 * Copyright 2024 LeoDrive.ai, Inc. All rights reserved.
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

#include "mapora/mapora.hpp"

#include <Eigen/Geometry>
#include <mapora/point_types.hpp>
#include <mapora/utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <liblas/liblas.hpp>
#include <pcl/io/pcd_io.h>

#include <cstdint>
#include <execution>
#include <memory>
#include <string>
#include <vector>

namespace {
const std::uint32_t QOS_HISTORY_DEPTH = 10;
}

namespace mapora {
Mapora::Mapora(const rclcpp::NodeOptions &options) : Node("mapora") {
  pub_ptr_cloud_current_ =
      this->create_publisher<PointCloud2>("cloud_current", 10);
  pub_ptr_path =
      this->create_publisher<nav_msgs::msg::Path>("path", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  this->declare_parameter("r_x", 0.0);
  this->declare_parameter("r_y", 0.0);
  this->declare_parameter("r_z", 0.0);
  this->declare_parameter("pcap_dir_path", "");
  this->declare_parameter("pose_txt_path", "");
  this->declare_parameter("project_namespace", "mapora_clouds");
  this->declare_parameter("max_point_distance_from_lidar", 60.0);
  this->declare_parameter("min_point_distance_from_lidar", 3.0);
  this->declare_parameter("las_export_directory", "/projects/mapora_ws/src");
  this->declare_parameter("lidar_model", "hesai_xt32");

  imu2lidar_roll = this->get_parameter("r_x").as_double();
  imu2lidar_pitch = this->get_parameter("r_y").as_double();
  imu2lidar_yaw = this->get_parameter("r_z").as_double();
  pcap_dir_path_ = this->get_parameter("pcap_dir_path").as_string();
  pose_txt_path_ = this->get_parameter("pose_txt_path").as_string();
  project_namespace_ = this->get_parameter("project_namespace").as_string();
  max_point_distance_from_lidar_ =
      this->get_parameter("max_point_distance_from_lidar").as_double();
  min_point_distance_from_lidar_ =
      this->get_parameter("min_point_distance_from_lidar").as_double();
  las_export_dir = this->get_parameter("las_export_directory").as_string();
  lidar_model = this->get_parameter("lidar_model").as_string();

  // Pose extraction
  RCLCPP_INFO(this->get_logger(),
              "Transform Provider started with the .txt file in: %s",
              pose_txt_path_.c_str());
  transform_provider =
      std::make_shared<transform_provider::TransformProvider>(
          pose_txt_path_);
  transform_provider->process();
  RCLCPP_INFO(this->get_logger(), "Transform Provider is DONE.");

  // Point Cloud Extraction
  RCLCPP_INFO(
      this->get_logger(),
      "Point Cloud Provider started with the PCAP files in : %s",
      pcap_dir_path_.c_str());
  points_provider_ = std::make_shared<points_provider::PointsProvider>(
      pcap_dir_path_, lidar_model);
  points_provider_->process();

  std::function<void(const Points &)> callback = std::bind(
      &Mapora::callback_cloud_surround_out, this, std::placeholders::_1);
  // time
  auto whole_process_start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < points_provider_->paths_pcaps_.size(); i++) {
    auto start = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "Processing the %d. pcap file.", i);
    points_provider_->process_pcaps_into_clouds(callback, i, 1,
                                                min_point_distance_from_lidar_,
                                                max_point_distance_from_lidar_);
    //    future_process = std::async(std::launch::async, &Mapora::process,
    //    this);
    process();
    clouds.erase(clouds.begin(), clouds.end());
    file_counter++;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    RCLCPP_INFO(this->get_logger(), "%zu. cloud process took %ld seconds.",
                file_counter, duration.count() / 1000);
    RCLCPP_INFO(this->get_logger(), "------------------------------------------"
                                    "-------------------------------");
  }
  auto whole_process_stop = std::chrono::high_resolution_clock::now();
  auto whole_process_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          whole_process_stop - whole_process_start);
  RCLCPP_INFO(this->get_logger(), "Whole process took %ld seconds.",
              whole_process_duration.count() / 1000);
  rclcpp::shutdown();
}

void Mapora::process() {
  RCLCPP_INFO(this->get_logger(),
              "Calibration angles, r_x: %f, r_y: %f, r_z: %f degrees",
              imu2lidar_roll, imu2lidar_pitch, imu2lidar_yaw);
  RCLCPP_INFO(this->get_logger(), "Mapora begins.");

  std::string path = las_export_dir;
  std::string name_las = path + project_namespace_ + "_global_" +
                         std::to_string(file_counter) + ".las";
  std::ofstream ofs;
  ofs.open(name_las, std::ios::out | std::ios::binary);
  liblas::Header las_header;
  las_header.SetScale(0.001, 0.001, 0.001);
  las_header.SetDataFormatId(liblas::ePointFormat0);
  liblas::Writer writer(ofs, las_header);

  auto thing_to_cloud = [](const points_provider::PointsProvider::Points
                               &points_bad,
                           const std::string &frame_id) {
    using CloudModifier =
        point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZI>;
    PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
    CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
    cloud_modifier_current.resize(points_bad.size());
    std::transform(std::execution::par, points_bad.cbegin(), points_bad.cend(),
                   cloud_modifier_current.begin(),
                   [](const points_provider::PointsProvider::Point &point_bad) {
                     return point_types::PointXYZI{
                         point_bad.x, point_bad.y, point_bad.z,
                         static_cast<float>(point_bad.intensity)};
                   });
    return cloud_ptr_current;
  };

  nav_msgs::msg::Path path_;
  path_.header.frame_id = "map";
  path_.header.stamp = this->get_clock()->now();
  path_.poses.resize(transform_provider->poses_.size());

  for (auto &pose : transform_provider->poses_) {
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header.frame_id = "map";
    poseStamped.header.stamp = this->get_clock()->now();
    poseStamped.pose = pose.pose_with_covariance.pose;
    path_.poses.push_back(poseStamped);
  }
  pub_ptr_path->publish(path_);

  points_provider::PointsProvider::Points cloud_all;

  for (auto &cloud : clouds) {
    Points cloud_trans = transform_points(cloud);
    auto cloud_ptr_current = thing_to_cloud(cloud_trans, "map");
    pub_ptr_cloud_current_->publish(*cloud_ptr_current);
    cloud_all.insert(cloud_all.end(), cloud_trans.begin(), cloud_trans.end());

    //    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (auto &point : cloud_trans) {
      liblas::Point las_point(&las_header);
      las_point.SetX(point.x);
      las_point.SetY(point.y);
      las_point.SetZ(point.z);
      las_point.SetIntensity(point.intensity);
      writer.WritePoint(las_point);
    }
  }

  cloud_all.clear();
}

void Mapora::callback_cloud_surround_out(
    const Mapora::Points &points_surround) {
  //  pub_ptr_basic_cloud_current_->publish(*points_to_cloud(points_surround,
  //  "map")); std::this_thread::sleep_for(std::chrono::milliseconds(150));
  clouds.push_back(points_surround);
}

sensor_msgs::msg::PointCloud2::SharedPtr
Mapora::points_to_cloud(const Mapora::Points &points_bad,
                        const std::string &frame_id) {
  using CloudModifier =
      point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZIR>;
  PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
  CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
  cloud_modifier_current.resize(points_bad.size());
  std::transform(
      std::execution::par, points_bad.cbegin(), points_bad.cend(),
      cloud_modifier_current.begin(),
      [](const points_provider::PointsProvider::Point &point_bad) {
        return point_types::PointXYZIR{
            point_bad.x, point_bad.y, point_bad.z,
            static_cast<uint32_t>(point_bad.intensity),
            //        static_cast<float>(point_bad.stamp_nanoseconds),
            //        static_cast<float>(point_bad.horizontal_angle),
            static_cast<uint32_t>(point_bad.ring)};
      });
  return cloud_ptr_current;
}

Mapora::Points Mapora::transform_points(Mapora::Points &cloud) {
  Points filtered_points;
  //  filtered_points.resize(cloud.size());

  auto last_pose = transform_provider->poses_.back();
  std::copy_if(
      cloud.cbegin(), cloud.cend(), std::back_inserter(filtered_points),
      [&last_pose](const points_provider::PointsProvider::Point &point) {
        return !(point.stamp_unix_seconds > last_pose.stamp_unix_seconds ||
                 (point.stamp_unix_seconds == last_pose.stamp_unix_seconds &&
                  point.stamp_nanoseconds > last_pose.stamp_nanoseconds));
      });

  Points cloud_trans;
  cloud_trans.resize(filtered_points.size());

  std::transform(
      std::execution::par, filtered_points.cbegin(), filtered_points.cend(),
      cloud_trans.begin(),
      [this](const points_provider::PointsProvider::Point &point) {
        if ((point.stamp_unix_seconds >
             transform_provider->poses_.back().stamp_unix_seconds) ||
            (point.stamp_unix_seconds ==
                 transform_provider->poses_.back()
                     .stamp_unix_seconds &&
             point.stamp_nanoseconds >
                 transform_provider->poses_.back()
                     .stamp_nanoseconds)) {
          return last_point;
        }

        points_provider::PointsProvider::Point point_trans;

        mapora::transform_provider::TransformProvider::Pose
            pose = this->transform_provider->get_pose_at(
                point.stamp_unix_seconds, point.stamp_nanoseconds);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose.pose_with_covariance.pose;
        pose_stamped.header.frame_id = "map";

        const auto &pose_ori = pose.pose_with_covariance.pose.orientation;
        Eigen::Quaterniond quat(pose_ori.w, pose_ori.x, pose_ori.y, pose_ori.z);

        Eigen::Affine3d affine_sensor2map(Eigen::Affine3d::Identity());

        Eigen::Affine3d affine_imu2lidar(Eigen::Affine3d::Identity());
        affine_imu2lidar.matrix().topLeftCorner<3, 3>() =
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(
                                  -(imu2lidar_yaw - pose.meridian_convergence)),
                              Eigen::Vector3d::UnitZ())
                .toRotationMatrix() *
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_pitch),
                              Eigen::Vector3d::UnitY())
                .toRotationMatrix() *
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_roll),
                              Eigen::Vector3d::UnitX())
                .toRotationMatrix();

        Eigen::Affine3d affine_imu2lidar_enu;
        affine_imu2lidar_enu.matrix().topLeftCorner<3, 3>() =
            utils::Utils::ned2enu_converter_for_matrices(
                affine_imu2lidar.matrix().topLeftCorner<3, 3>());

        affine_sensor2map.matrix().topLeftCorner<3, 3>() =
            quat.toRotationMatrix() * affine_imu2lidar_enu.rotation();

        auto &pose_pos = pose_stamped.pose.position;
        affine_sensor2map.matrix().topRightCorner<3, 1>() << pose_pos.x,
            pose_pos.y, pose_pos.z;

        Eigen::Vector4d vec_point_in(point.x, point.y, point.z, 1.0);
        Eigen::Vector4d vec_point_trans =
            affine_sensor2map.matrix() * vec_point_in;

        point_trans.x = static_cast<float>(vec_point_trans(0));
        point_trans.y = static_cast<float>(vec_point_trans(1));
        point_trans.z = static_cast<float>(vec_point_trans(2));
        point_trans.ring = point.ring;
        point_trans.horizontal_angle = point.horizontal_angle;
        point_trans.intensity = point.intensity;
        point_trans.stamp_unix_seconds = point.stamp_unix_seconds;
        point_trans.stamp_nanoseconds = point.stamp_nanoseconds;

        last_point = point_trans;

        return point_trans;
      });

  return cloud_trans;
}

} // namespace mapora
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mapora::Mapora)
