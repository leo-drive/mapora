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

#ifndef BUILD_MAPORA_ROSBAG_HPP
#define BUILD_MAPORA_ROSBAG_HPP

#include <mapora/mapora.hpp>
#include <rclcpp/rclcpp.hpp>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>

namespace mapora
{
class MaporaRosbag : public rclcpp::Node
{
public:
  explicit MaporaRosbag(const rclcpp::NodeOptions & options);

  // params
  std::string project_namespace_;
  double imu2lidar_roll;
  double imu2lidar_pitch;
  double imu2lidar_yaw;
  double imu2lidar_x;
  double imu2lidar_y;
  double imu2lidar_z;
  double max_point_distance_from_lidar_;
  double min_point_distance_from_lidar_;
  std::string las_export_dir;
  double err_pos_x_;
  double err_pos_y_;
  double err_pos_z_;
  double err_rot_x_;
  double err_rot_y_;
  double err_rot_z_;
  std::string rosbag_path_;
  std::string point_cloud_topic_;
  std::string gnss_topic_;
  std::string imu_topic_;
  std::string twist_with_covariance_stamped_topic_;
  bool correct_distortions_;
  std::string base_link_tf_name_;
  std::string lidar_tf_name_;
  int save_point_cloud_after_msgs_;
  std::string point_cloud_timestamp_field_;
private:

  std::unique_ptr<tf2_ros::TransformBroadcaster> sensor_kit_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> lidar_tf_broadcaster_;

  geometry_msgs::msg::TransformStamped sensor_kit_tf;
  geometry_msgs::msg::TransformStamped lidar_tf;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
  std::string base_link_frame_ = "base_link";

  double global_x, global_y, global_z;
  bool pose_accuracy_ok;
  bool point_cloud_init = false;

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_{tf2_buffer_};

  float v;
  float w;


  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);
  void onTwistWithCovarianceStamped(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg);
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
  bool undistortPointCloud(const tf2::Transform & tf2_base_link_to_sensor,
                           sensor_msgs::msg::PointCloud2 & points);
  sensor_msgs::msg::PointCloud2 onPointCloud(sensor_msgs::msg::PointCloud2 points_msg);
};
}  // namespace mapora




#endif //BUILD_MAPORA_ROSBAG_HPP
