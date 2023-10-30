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

#include <mapora/mapora.hpp>
#include <mapora/mapora_rosbag.hpp>
#include <mapora/utils.hpp>

#include <Eigen/Geometry>
#include <liblas/liblas.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

#include <fstream>
#include <execution>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>


namespace
{
const std::uint32_t QOS_HISTORY_DEPTH = 10;
}

namespace mapora {
MaporaRosbag::MaporaRosbag(const rclcpp::NodeOptions &options)
  : Node("mapora_rosbag", options)
{
  this->declare_parameter("project_namespace", "mapora_clouds");
  this->declare_parameter("max_point_distance_from_lidar", 60.0);
  this->declare_parameter("min_point_distance_from_lidar", 3.0);
  this->declare_parameter("las_export_directory", "/projects/mapora_ws/src");
  this->declare_parameter("save_point_cloud_after_msgs", 20000);
  this->declare_parameter("r_x", 0.0);
  this->declare_parameter("r_y", 0.0);
  this->declare_parameter("r_z", 180.0);
  this->declare_parameter("t_x", 0.0);
  this->declare_parameter("t_y", 0.0);
  this->declare_parameter("t_z", 0.0);
  this->declare_parameter("err_pos_x", 0.08);
  this->declare_parameter("err_pos_y", 0.08);
  this->declare_parameter("err_pos_z", 0.16);
  this->declare_parameter("err_rot_x", 0.06);
  this->declare_parameter("err_rot_y", 0.06);
  this->declare_parameter("err_rot_z", 0.18);
  this->declare_parameter("rosbag_path", "");
  this->declare_parameter("point_cloud_topic", "/point_cloud_topic");
  this->declare_parameter("imu_topic", "/imu_topic");
  this->declare_parameter("gnss_topic", "/gnss_topic");
  this->declare_parameter("twist_with_covariance_stamped_topic", "/twist_topic");

  this->declare_parameter("correct_distortions", false);
  this->declare_parameter("base_link_tf_name", "base_link");
  this->declare_parameter("lidar_tf_name", "lidar_link");
  this->declare_parameter("point_cloud_timestamp_field", "timestamp");

  project_namespace_ = this->get_parameter("project_namespace").as_string();
  max_point_distance_from_lidar_ = this->get_parameter("max_point_distance_from_lidar").as_double();
  min_point_distance_from_lidar_ = this->get_parameter("min_point_distance_from_lidar").as_double();
  las_export_dir = this->get_parameter("las_export_directory").as_string();
  save_point_cloud_after_msgs_ = this->get_parameter("save_point_cloud_after_msgs").as_int();
  imu2lidar_roll = this->get_parameter("r_x").as_double();
  imu2lidar_pitch = this->get_parameter("r_y").as_double();
  imu2lidar_yaw = this->get_parameter("r_z").as_double();
  imu2lidar_x = this->get_parameter("t_x").as_double();
  imu2lidar_y = this->get_parameter("t_y").as_double();
  imu2lidar_z = this->get_parameter("t_z").as_double();
  err_pos_x_ = this->get_parameter("err_pos_x").as_double();
  err_pos_y_ = this->get_parameter("err_pos_y").as_double();
  err_pos_z_ = this->get_parameter("err_pos_z").as_double();
  err_rot_x_ = this->get_parameter("err_rot_x").as_double();
  err_rot_y_ = this->get_parameter("err_rot_y").as_double();
  err_rot_z_ = this->get_parameter("err_rot_z").as_double();
  point_cloud_topic_ = this->get_parameter("point_cloud_topic").as_string();
  rosbag_path_ = this->get_parameter("rosbag_path").as_string();
  gnss_topic_ = this->get_parameter("gnss_topic").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  twist_with_covariance_stamped_topic_ = this->get_parameter("twist_with_covariance_stamped_topic").as_string();
  correct_distortions_ = this->get_parameter("correct_distortions").as_bool();
  base_link_tf_name_ = this->get_parameter("base_link_tf_name").as_string();
  lidar_tf_name_ = this->get_parameter("lidar_tf_name").as_string();
  point_cloud_timestamp_field_ = this->get_parameter("point_cloud_timestamp_field").as_string();

  sensor_kit_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  lidar_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "Starting Mapora Rosbag...");

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = rosbag_path_;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error opening bag file: " << e.what());
    rclcpp::shutdown();
    return;
  }

  const std::string & point_cloud_topic_ref = point_cloud_topic_;
  const std::string & gnss_topic_ref = gnss_topic_;
  const std::string &imu_topic_ref = imu_topic_;
  const std::string &twist_with_covariance_ref = twist_with_covariance_stamped_topic_;

  const auto & topics = reader.get_metadata().topics_with_message_count;
  const auto iter_topic = std::find_if(
    topics.begin(), topics.end(), [
      &point_cloud_topic_ref, &gnss_topic_ref,
      &imu_topic_ref, &twist_with_covariance_ref]
      (const auto & topic) {
      return topic.topic_metadata.name == point_cloud_topic_ref ||
             topic.topic_metadata.name == gnss_topic_ref ||
             topic.topic_metadata.name == imu_topic_ref ||
             topic.topic_metadata.name == twist_with_covariance_ref;
    });

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_point_cloud;
  rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_imu;
  rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serialization_gnss;
  rclcpp::Serialization<geometry_msgs::msg::TwistWithCovarianceStamped> serialization_twist;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;

  int msg_counter = 0;
  int iterate_counter = 0;
  int msg_counter_stop = 0;

  while (rclcpp::ok() && reader.has_next()) {


    std::ofstream ofs;
    std::string las_name = las_export_dir + project_namespace_ + "_" + std::to_string(iterate_counter) + ".las";
    RCLCPP_INFO(this->get_logger(), "Saving the point cloud: %d\t to %s", iterate_counter, las_name.c_str());

    ofs.open( las_name,std::ios::out | std::ios::binary);
    liblas::Header las_header;
    las_header.SetScale(0.01, 0.01, 0.01);
    las_header.SetDataFormatId(liblas::ePointFormat0);
    liblas::Writer writer(ofs, las_header);

    msg_counter_stop = msg_counter;

    while (reader.has_next() && rclcpp::ok()) {
      msg = reader.read_next();
      if (msg_counter < msg_counter_stop) {
        continue;
      };

      pose_stamped.header.stamp = this->get_clock()->now();
      pose_stamped.header.frame_id = base_link_tf_name_;

      // if msg is imu
      if (msg->topic_name == imu_topic_) {
        using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;
        auto msg_imu = std::make_shared<sensor_msgs::msg::Imu>();

        rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
        serialization_imu.deserialize_message(&extracted_serialized_msg, &(*msg_imu));

        if (correct_distortions_) {
          onImu(msg_imu);
        }
        sensor_kit_tf.transform.rotation.x = msg_imu->orientation.x;
        sensor_kit_tf.transform.rotation.y = msg_imu->orientation.y;
        sensor_kit_tf.transform.rotation.z = msg_imu->orientation.z;
        sensor_kit_tf.transform.rotation.w = msg_imu->orientation.w;

        pose_stamped.pose.pose.orientation = msg_imu->orientation;
        pose_stamped.pose.covariance[21] = msg_imu->orientation_covariance[0];
        pose_stamped.pose.covariance[28] = msg_imu->orientation_covariance[5];
        pose_stamped.pose.covariance[35] = msg_imu->orientation_covariance[9];
      }

      // If msg is navsatfix
      if (msg->topic_name == gnss_topic_) {
        using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;
        auto msg_gnss = std::make_shared<sensor_msgs::msg::NavSatFix>();

        rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
        serialization_gnss.deserialize_message(&extracted_serialized_msg, &(*msg_gnss));

        int zone;
        bool northp;
        GeographicLib::UTMUPS::Forward(
          msg_gnss->latitude, msg_gnss->longitude, zone, northp, global_x, global_y);

        global_z = msg_gnss->altitude;
        pose_stamped.pose.pose.position.x = global_x;
        pose_stamped.pose.pose.position.y = global_y;
        pose_stamped.pose.pose.position.z = global_z;

        pose_stamped.pose.covariance[0] = msg_gnss->position_covariance[0];
        pose_stamped.pose.covariance[7] = msg_gnss->position_covariance[5];
        pose_stamped.pose.covariance[14] = msg_gnss->position_covariance[9];

        if (
          std::sqrt(pose_stamped.pose.covariance[0]) > err_pos_x_ ||
          std::sqrt(pose_stamped.pose.covariance[7]) > err_pos_y_ ||
          std::sqrt(pose_stamped.pose.covariance[14]) > err_pos_z_ ||
          pose_stamped.pose.covariance[21] > utils::Utils::deg_to_rad(err_rot_x_) ||
          pose_stamped.pose.covariance[28] > utils::Utils::deg_to_rad(err_rot_y_) ||
          pose_stamped.pose.covariance[35] > utils::Utils::deg_to_rad(err_rot_z_)) {
          pose_accuracy_ok = false;
        } else {
          pose_accuracy_ok = true;
        }
      }

      if (msg->topic_name == twist_with_covariance_stamped_topic_) {
        using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;
        auto msg_twist = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();

        rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
        serialization_twist.deserialize_message(&extracted_serialized_msg, &(*msg_twist));

        if (correct_distortions_) {
          onTwistWithCovarianceStamped(msg_twist);
        }
      }

      // If msg is point_cloud
      if (msg->topic_name == point_cloud_topic_ && pose_accuracy_ok) {
        using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;
        auto msg_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

        rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
        serialization_point_cloud.deserialize_message(&extracted_serialized_msg, &(*msg_cloud));

        if (!point_cloud_init) {
          tf2::Quaternion sensor_kit2lidar;
          sensor_kit2lidar.setRPY(
            utils::Utils::deg_to_rad(imu2lidar_roll), utils::Utils::deg_to_rad(imu2lidar_pitch),
            utils::Utils::deg_to_rad(imu2lidar_yaw));
          lidar_tf.header.frame_id = base_link_tf_name_;
          lidar_tf.header.stamp = this->get_clock()->now();
          lidar_tf.child_frame_id = "lidar";
          lidar_tf.transform.translation.x = imu2lidar_x;
          lidar_tf.transform.translation.y = imu2lidar_y;
          lidar_tf.transform.translation.z = imu2lidar_z;
          lidar_tf.transform.rotation.x = sensor_kit2lidar.x();
          lidar_tf.transform.rotation.y = sensor_kit2lidar.y();
          lidar_tf.transform.rotation.z = sensor_kit2lidar.z();
          lidar_tf.transform.rotation.w = sensor_kit2lidar.w();
          lidar_tf_broadcaster_->sendTransform(lidar_tf);
          point_cloud_init = true;
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::Transform tf2_base_link_to_sensor{};

        if (correct_distortions_) {
          sensor_msgs::msg::PointCloud2 cloud;
          cloud.header.stamp = this->get_clock()->now();
          cloud.header.frame_id = lidar_tf_name_;
          cloud.height = msg_cloud->height;
          cloud.width = msg_cloud->width;
          cloud.data = msg_cloud->data;
          cloud.fields = msg_cloud->fields;
          cloud.is_bigendian = msg_cloud->is_bigendian;
          cloud.is_dense = msg_cloud->is_dense;
          cloud.point_step = msg_cloud->point_step;
          cloud.row_step = msg_cloud->row_step;

          tf2::Quaternion sensor_kit2lidar;
          sensor_kit2lidar.setRPY(
            imu2lidar_roll * M_PI / 180, imu2lidar_pitch * M_PI / 180, imu2lidar_yaw * M_PI / 180);
          lidar_tf.header.frame_id = base_link_tf_name_;
          lidar_tf.header.stamp = this->get_clock()->now();
          lidar_tf.child_frame_id = lidar_tf_name_;
          lidar_tf.transform.translation.x = imu2lidar_x;
          lidar_tf.transform.translation.y = imu2lidar_y;
          lidar_tf.transform.translation.z = imu2lidar_z;
          lidar_tf.transform.rotation.x = sensor_kit2lidar.x();
          lidar_tf.transform.rotation.y = sensor_kit2lidar.y();
          lidar_tf.transform.rotation.z = sensor_kit2lidar.z();
          lidar_tf.transform.rotation.w = sensor_kit2lidar.w();
          lidar_tf_broadcaster_->sendTransform(lidar_tf);

          transformed_cloud = onPointCloud(cloud);
          getTransform(msg_cloud->header.frame_id, base_link_frame_, &tf2_base_link_to_sensor);
        }

        sensor_msgs::PointCloud2Modifier modifier(transformed_cloud);

        modifier.resize(msg_cloud->height * msg_cloud->width);
        modifier.setPointCloud2Fields(
          4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
          sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        // filled pointcloud2
        sensor_msgs::PointCloud2ConstIterator<float> child_x(*msg_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> child_y(*msg_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> child_z(*msg_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> child_i(*msg_cloud, "intensity");

        for (size_t i = 0; i < msg_cloud->height * msg_cloud->width;
             ++i, ++child_x, ++child_y, ++child_z, ++child_i) {
          double transformed_x, transformed_y, transformed_z;
          double x, y, z;
          x = *child_x;
          y = *child_y;
          z = *child_z;

          // point distance filter 30m
          if (
            std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2)) >
            max_point_distance_from_lidar_ ||
            min_point_distance_from_lidar_ >
            std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2))) {
            continue;
          }

          Eigen::Quaterniond quat_ins_to_map(
            pose_stamped.pose.pose.orientation.w, pose_stamped.pose.pose.orientation.x,
            pose_stamped.pose.pose.orientation.y, pose_stamped.pose.pose.orientation.z);

          Eigen::Affine3d affine_imu2lidar(Eigen::Affine3d::Identity());
          affine_imu2lidar.matrix().topLeftCorner<3, 3>() =
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_yaw), Eigen::Vector3d::UnitZ())
              .toRotationMatrix() *
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_pitch), Eigen::Vector3d::UnitY())
              .toRotationMatrix() *
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_roll), Eigen::Vector3d::UnitX())
              .toRotationMatrix();

          Eigen::Affine3d affine_sensor2map(Eigen::Affine3d::Identity());
          affine_sensor2map.matrix().topLeftCorner<3, 3>() =
            quat_ins_to_map.toRotationMatrix() * affine_imu2lidar.rotation();

          auto & pose_pos = pose_stamped.pose.pose.position;
          affine_sensor2map.matrix().topRightCorner<3, 1>() << pose_pos.x, pose_pos.y, pose_pos.z;

          Eigen::Vector4d vec_point_in_first(x, y, z, 1.0);
          // create a 3D vector for transformed point to the map position and rotation.
          Eigen::Vector4d vec_point_trans = affine_sensor2map.matrix() * vec_point_in_first;

          transformed_x = vec_point_trans(0);
          transformed_y = vec_point_trans(1);
          transformed_z = vec_point_trans(2);

          liblas::Point las_point(&las_header);
          las_point.SetX(transformed_x);
          las_point.SetY(transformed_y);
          las_point.SetZ(transformed_z);
          las_point.SetIntensity(*child_i);
          writer.WritePoint(las_point);
        }
      }
      if (msg_counter > 0 && msg_counter % save_point_cloud_after_msgs_ == 0){
        msg_counter_stop = msg_counter;
        msg_counter = 0;
        iterate_counter++;
        break;
      }
      msg_counter++;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Done.");
  reader.close();
  rclcpp::shutdown();
}  // constructor ends

bool MaporaRosbag::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return false;
  }
  return true;
}


void MaporaRosbag::onTwistWithCovarianceStamped(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header = twist_msg->header;
  msg.twist = twist_msg->twist.twist;
  twist_queue_.push_back(msg);

  while (!twist_queue_.empty()) {
    // for replay rosbag
    if (rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(twist_msg->header.stamp)) {
      twist_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(twist_queue_.front().header.stamp) <
      rclcpp::Time(twist_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_queue_.pop_front();
    }
    break;
  }
}


void MaporaRosbag::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{

  tf2::Transform tf2_imu_link_to_base_link{};
  getTransform(base_link_frame_, imu_msg->header.frame_id, &tf2_imu_link_to_base_link);
  geometry_msgs::msg::TransformStamped::SharedPtr tf_base2imu_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf_base2imu_ptr->transform.rotation = tf2::toMsg(tf2_imu_link_to_base_link.getRotation());

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_base2imu_ptr);
  transformed_angular_velocity.header = imu_msg->header;
  angular_velocity_queue_.push_back(transformed_angular_velocity);

  while (!angular_velocity_queue_.empty()) {
    // for replay rosbag
    if (
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) >
      rclcpp::Time(imu_msg->header.stamp)) {
      angular_velocity_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) <
      rclcpp::Time(imu_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      angular_velocity_queue_.pop_front();
    }
    break;
  }
}


sensor_msgs::msg::PointCloud2 MaporaRosbag::onPointCloud(sensor_msgs::msg::PointCloud2 points_msg)
{
  tf2::Transform tf2_base_link_to_sensor{};
  getTransform(points_msg.header.frame_id, base_link_frame_, &tf2_base_link_to_sensor);
  undistortPointCloud(tf2_base_link_to_sensor, points_msg);
  return points_msg;
}


bool MaporaRosbag::undistortPointCloud(
  const tf2::Transform & tf2_base_link_to_sensor, sensor_msgs::msg::PointCloud2 & points)
{
  // if point cloud data empty
  if (points.data.empty() || twist_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "input_pointcloud->points or twist_queue_ is empty.");
    return false;
  }

  // check there are any "timestamp" field in the point cloud fields
  auto time_stamp_field_it = std::find_if(
    std::cbegin(points.fields), std::cend(points.fields),
    [this](const sensor_msgs::msg::PointField & field) {
      return field.name == "timestamp";
    });
  if (time_stamp_field_it == points.fields.cend()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "Required field time stamp doesn't exist in the point cloud.");
    return false;
  }

  // create point cloud iterators
  sensor_msgs::PointCloud2Iterator<float> it_x(points, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(points, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(points, "z");
  sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(points, point_cloud_timestamp_field_);

  float theta{0.0f};  // azimuth angle
  float x{0.0f};  // x of point
  float y{0.0f};  // y of point
  double prev_time_stamp_sec{*it_time_stamp};
  const double first_point_time_stamp_sec{*it_time_stamp};  // first point's timestamp

  auto twist_it = std::lower_bound(  // Returns an iterator pointing to the first element in the range
    std::begin(twist_queue_), std::end(twist_queue_), first_point_time_stamp_sec,
    [](const geometry_msgs::msg::TwistStamped & x, const double t) {
      return rclcpp::Time(x.header.stamp).seconds() < t;
    });
  twist_it = twist_it == std::end(twist_queue_) ? std::end(twist_queue_) - 1 : twist_it;

  decltype(angular_velocity_queue_)::iterator imu_it;
  if (!angular_velocity_queue_.empty()) {
    imu_it = std::lower_bound(
      std::begin(angular_velocity_queue_), std::end(angular_velocity_queue_),
      first_point_time_stamp_sec, [](const geometry_msgs::msg::Vector3Stamped & x, const double t) {
        return rclcpp::Time(x.header.stamp).seconds() < t;
      });
    imu_it =
      imu_it == std::end(angular_velocity_queue_) ? std::end(angular_velocity_queue_) - 1 : imu_it;
  }

  const tf2::Transform tf2_base_link_to_sensor_inv{tf2_base_link_to_sensor.inverse()};

  // For performance, do not instantiate `rclcpp::Time` inside of the for-loop
  double twist_stamp = rclcpp::Time(twist_it->header.stamp).seconds();

  // For performance, instantiate outside of the for-loop
  tf2::Quaternion baselink_quat{};
  tf2::Transform baselink_tf_odom{};
  tf2::Vector3 point{};
  tf2::Vector3 undistorted_point{};

  // For performance, avoid transform computation if unnecessary
  bool need_transform = points.header.frame_id != base_link_frame_;

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp) {

    // Choose twist data according to the time.
    while (twist_it != std::end(twist_queue_) - 1 && *it_time_stamp > twist_stamp) {
      ++twist_it;
      twist_stamp = rclcpp::Time(twist_it->header.stamp).seconds();
    }

    v = static_cast<float>(twist_it->twist.linear.x);
    w = static_cast<float>(twist_it->twist.angular.z);

    if (std::abs(*it_time_stamp - twist_stamp) > 0.1) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 1000 /* ms */,
        "twist time_stamp is too late. Could not interpolate.");
      v = 0.0f;
      w = 0.0f;
    }

    if (!angular_velocity_queue_.empty()) {
      // For performance, do not instantiate `rclcpp::Time` inside of the for-loop
      double imu_stamp = rclcpp::Time(imu_it->header.stamp).seconds();

      for (;
        (imu_it != std::end(angular_velocity_queue_) - 1 &&
         *it_time_stamp > rclcpp::Time(imu_it->header.stamp).seconds());
        ++imu_it) {
      }

      while (imu_it != std::end(angular_velocity_queue_) - 1 && *it_time_stamp > imu_stamp) {
        ++imu_it;
        imu_stamp = rclcpp::Time(imu_it->header.stamp).seconds();
      }

      if (std::abs(*it_time_stamp - imu_stamp) > 0.1) {
        RCLCPP_WARN_STREAM_THROTTLE(
          get_logger(), *get_clock(), 10000 /* ms */,
          "imu time_stamp is too late. Could not interpolate.");
      } else {
        w = static_cast<float>(imu_it->vector.z);
      }
    }

    const auto time_offset = static_cast<float>(*it_time_stamp - prev_time_stamp_sec);

    point.setValue(*it_x, *it_y, *it_z);

    if (need_transform) {
      point = tf2_base_link_to_sensor_inv * point;
    }

    theta += w * time_offset;
    baselink_quat.setValue(
      0, 0, sin(theta * 0.5f),
      cos(theta * 0.5f));  // baselink_quat.setRPY(0.0, 0.0, theta);
    const float dis = v * time_offset;
    x += dis * cos(theta);
    y += dis * sin(theta);

    baselink_tf_odom.setOrigin(tf2::Vector3(x, y, 0.0));
    baselink_tf_odom.setRotation(baselink_quat);

    undistorted_point = baselink_tf_odom * point;

    if (need_transform) {
      undistorted_point = tf2_base_link_to_sensor * undistorted_point;
    }

    *it_x = static_cast<float>(undistorted_point.getX());
    *it_y = static_cast<float>(undistorted_point.getY());
    *it_z = static_cast<float>(undistorted_point.getZ());

    prev_time_stamp_sec = *it_time_stamp;

  }
  return true;
}


}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mapora::MaporaRosbag)