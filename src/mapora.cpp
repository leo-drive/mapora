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

#include "mapora/mapora.hpp"

#include <Eigen/Geometry>
#include <mapora/point_xyzi.hpp>
#include <mapora/utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <liblas/liblas.hpp>
#include "mapora/points_provider_velodyne_vlp16.hpp"
#include "mapora/transform_provider_applanix.hpp"

#include <fstream>
#include <execution>
#include <memory>
#include <string>
#include <vector>


namespace
{
const std::uint32_t QOS_HISTORY_DEPTH = 10;
}

size_t file_counter = 0;

namespace mapora
{
Mapora::Mapora(const rclcpp::NodeOptions & options)
  : Node("mapora", options)
{
  pub_ptr_cloud_current_ = this->create_publisher<PointCloud2>("cloud_current", 10);
  pub_ptr_path_applanix_ = this->create_publisher<nav_msgs::msg::Path>("path_applanix", 10);
  tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  this->declare_parameter("r_x", 0.0);
  this->declare_parameter("r_y", 0.0);
  this->declare_parameter("r_z", 0.0);
  this->declare_parameter("pcap_dir_path", "");
  this->declare_parameter("pose_txt_path", "");
  this->declare_parameter("project_namespace", "mapora_clouds");
  this->declare_parameter("max_point_distance_from_lidar", 60.0);
  this->declare_parameter("min_point_distance_from_lidar", 3.0);
  this->declare_parameter("las_export_directory", "/projects/mapora_ws/src");

  imu2lidar_roll = this->get_parameter("r_x").as_double();
  imu2lidar_pitch = this->get_parameter("r_y").as_double();
  imu2lidar_yaw = this->get_parameter("r_z").as_double();
  pcap_dir_path_ = this->get_parameter("pcap_dir_path").as_string();
  pose_txt_path_ = this->get_parameter("pose_txt_path").as_string();
  project_namespace_ = this->get_parameter("project_namespace").as_string();
  max_point_distance_from_lidar_ = this->get_parameter("max_point_distance_from_lidar").as_double();
  min_point_distance_from_lidar_ = this->get_parameter("min_point_distance_from_lidar").as_double();
  las_export_dir = this->get_parameter("las_export_directory").as_string();

  // Applanix Pose extraction
  RCLCPP_INFO(this->get_logger(), "Applanix Transform Provider started with the .txt file in: %s", pose_txt_path_.c_str());
  transform_provider_applanix = std::make_shared<transform_provider_applanix::TransformProviderApplanix>(
    pose_txt_path_);
  transform_provider_applanix->process();
  RCLCPP_INFO(this->get_logger(), "Applanix Transform Provider is DONE.");

  // Velodyne Point Cloud Extraction
  RCLCPP_INFO(this->get_logger(), "Velodyne Point Cloud Provider started with the PCAP files in : %s", pcap_dir_path_.c_str());
  points_provider_velodyne_vlp16 = std::make_shared<points_provider::PointsProviderVelodyneVlp16>(
    pcap_dir_path_);
  points_provider_velodyne_vlp16->process();

  std::function<void(const Points &)> callback =
    std::bind(&Mapora::callback_cloud_surround_out, this, std::placeholders::_1);
  // time
  auto whole_process_start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < points_provider_velodyne_vlp16->paths_pcaps_.size(); i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "Processing the %d. pcap file.", i);
    points_provider_velodyne_vlp16->process_pcaps_into_clouds(
      callback, i, 1,min_point_distance_from_lidar_, max_point_distance_from_lidar_);
//    future_process = std::async(std::launch::async, &Mapora::process, this);
    process();
    clouds.erase(clouds.begin(), clouds.end());
    file_counter++;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    RCLCPP_INFO(this->get_logger(), "%zu. cloud process took %ld seconds.", file_counter, duration.count()/1000);
    RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------------------");
  }
  auto whole_process_stop = std::chrono::high_resolution_clock::now();
  auto whole_process_duration = std::chrono::duration_cast<std::chrono::milliseconds>(whole_process_stop - whole_process_start);
  RCLCPP_INFO(this->get_logger(), "Whole process took %ld seconds.", whole_process_duration.count()/1000);
  rclcpp::shutdown();
}

void Mapora::process()
{
  RCLCPP_INFO(this->get_logger(), "Calibration angles, r_x: %f, r_y: %f, r_z: %f degrees",
              imu2lidar_roll, imu2lidar_pitch, imu2lidar_yaw);
  RCLCPP_INFO(this->get_logger(), "Mapora begins.");

  std::string path = las_export_dir;
  std::string name_las = path + project_namespace_ + "_global_" + std::to_string(file_counter) + ".las";
  std::ofstream ofs;
  ofs.open(name_las, std::ios::out | std::ios::binary);
  liblas::Header las_header;
  las_header.SetScale(0.001, 0.01, 0.001);
  las_header.SetDataFormatId(liblas::ePointFormat0);
  liblas::Writer writer(ofs, las_header);

  auto thing_to_cloud = [](
    const Points & points_bad,
    const std::string & frame_id) {
    using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZI>;
    PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
    CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
    cloud_modifier_current.resize(points_bad.size());
    std::transform(
      std::execution::par,
      points_bad.cbegin(),
      points_bad.cend(),
      cloud_modifier_current.begin(),
      [](const points_provider::PointsProviderVelodyneVlp16::Point & point_bad) {
        return point_types::PointXYZI{
          point_bad.x, point_bad.y, point_bad.z, static_cast<float>(point_bad.intensity)};
      });
    return cloud_ptr_current;
  };

  nav_msgs::msg::Path path_applanix;
  path_applanix.header.frame_id = "map";

  points_provider::PointsProviderVelodyneVlp16::Points cloud_all;

  path_applanix.poses.resize(transform_provider_applanix->poses_.size());
  std::cout << "transform_provider_applanix->poses_.size(): " << transform_provider_applanix->poses_.size() << std::endl;

  for (int i=0; i<transform_provider_applanix->poses_.size(); i++)
    path_applanix.poses[i].pose = transform_provider_applanix->poses_[i].pose_with_covariance.pose;
  pub_ptr_path_applanix_->publish(path_applanix);

  std::cout << "Point Cloud: " << file_counter << std::endl;
  auto process_cloud_single = [&](const points_provider::PointsProviderVelodyneVlp16::Points& cloud){

    points_provider::PointsProviderVelodyneVlp16::Points cloud_trans;
    cloud_trans.resize(cloud.size());

    std::transform(
      std::execution::par,
      cloud.cbegin(),
      cloud.cend(),
      cloud_trans.begin(),
      [this, &path_applanix](const points_provider::PointsProviderVelodyneVlp16::Point & point) {
        points_provider::PointsProviderVelodyneVlp16::Point point_trans;

        // position from applanix data is taken into pose below according to the stamps .
        pose = transform_provider_applanix->get_pose_at(point.stamp_unix_seconds, point.stamp_nanoseconds);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose.pose_with_covariance.pose;
        pose_stamped.header.frame_id = path_applanix.header.frame_id;
        pose_stamped.header.stamp.sec = point.stamp_unix_seconds;
        pose_stamped.header.stamp.nanosec = point.stamp_nanoseconds;

        const auto & pose_ori = pose.pose_with_covariance.pose.orientation;
        Eigen::Quaterniond quat_ins_to_map(pose_ori.w, pose_ori.x, pose_ori.y, pose_ori.z);
        Eigen::Affine3d affine_imu2lidar(Eigen::Affine3d::Identity());
        affine_imu2lidar.matrix().topLeftCorner<3, 3>() =
          Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_yaw), Eigen::Vector3d::UnitZ())
            .toRotationMatrix() *
          Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_pitch), Eigen::Vector3d::UnitY())
            .toRotationMatrix() *
          Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_roll), Eigen::Vector3d::UnitX())
            .toRotationMatrix();

        // sensor to map rotation is created to add translations and get the right rotation.
        Eigen::Affine3d affine_sensor2map(Eigen::Affine3d::Identity());

        affine_sensor2map.matrix().topLeftCorner<3, 3>() = quat_ins_to_map.toRotationMatrix().inverse() * affine_imu2lidar.rotation();

        // pose is added to the transformation matrix. - these were completed for every point in the pointclouds.
        auto & pose_pos = pose_stamped.pose.position;
        affine_sensor2map.matrix().topRightCorner<3, 1>() << pose_pos.x, pose_pos.y, pose_pos.z;

        // get the point's position w.r.t. the point cloud origin.
        Eigen::Vector4d vec_point_in_first(point.x,
                                           point.y,
                                           point.z,
                                           1.0);
        // create a 3D vector for transformed point to the map position and rotation.
        Eigen::Vector4d vec_point_trans = affine_sensor2map.matrix() * vec_point_in_first;

        point_trans.x = static_cast<double>(vec_point_trans(0));
        point_trans.y = static_cast<double>(vec_point_trans(1));
        point_trans.z = static_cast<double>(vec_point_trans(2));
        point_trans.intensity = point.intensity;

        return point_trans;
      });

    geometry_msgs::msg::TransformStamped t;
    t.transform.translation.x = pose.pose_with_covariance.pose.position.x;
    t.transform.translation.y = pose.pose_with_covariance.pose.position.y;
    t.transform.translation.z = pose.pose_with_covariance.pose.position.z;
    t.transform.rotation.x = pose.pose_with_covariance.pose.orientation.x;
    t.transform.rotation.y = pose.pose_with_covariance.pose.orientation.y;
    t.transform.rotation.z = pose.pose_with_covariance.pose.orientation.z;
    t.transform.rotation.w = pose.pose_with_covariance.pose.orientation.w;
    t.header.frame_id = "map";
    t.child_frame_id = "sensor_kit";

    tf_broadcaster_->sendTransform(t);

    auto cloud_ptr_current = thing_to_cloud(cloud_trans, "map");

    for (auto & point : cloud_trans) {
      liblas::Point las_point(&las_header);
      las_point.SetX(point.x);
      las_point.SetY(point.y);
      las_point.SetZ(point.z);
      las_point.SetIntensity(point.intensity);
      writer.WritePoint(las_point);
    }
    pub_ptr_cloud_current_->publish(*cloud_ptr_current);
  };

//   section 2
  for (auto & cloud : clouds) {
    process_cloud_single(cloud);
  }
}

void Mapora::callback_cloud_surround_out(const Mapora::Points & points_surround)
{
  clouds.push_back(points_surround);
}

sensor_msgs::msg::PointCloud2::SharedPtr Mapora::points_to_cloud(
  const Mapora::Points & points_bad, const std::string & frame_id)
{
  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZI>;
  PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
  CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
  cloud_modifier_current.resize(points_bad.size());
  std::transform(
    std::execution::par,
    points_bad.cbegin(),
    points_bad.cend(),
    cloud_modifier_current.begin(),
    [](const points_provider::PointsProviderVelodyneVlp16::Point & point_bad) {
      return point_types::PointXYZI{
        point_bad.x, point_bad.y, point_bad.z, static_cast<float>(point_bad.intensity)};
    });
  return cloud_ptr_current;
}

}  // namespace mapora
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mapora::Mapora)
