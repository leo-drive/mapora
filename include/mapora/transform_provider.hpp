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

#ifndef MAPORA__TRANSFORM_PROVIDER_APPLANIX_HPP_
#define MAPORA__TRANSFORM_PROVIDER_APPLANIX_HPP_

#include <boost/filesystem.hpp>
#include "mapora/csv.hpp"
#include <string>
#include <memory>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace mapora::transform_provider {
namespace fs = boost::filesystem;
class TransformProvider {
public:
  using SharedPtr = std::shared_ptr<TransformProvider>;
  using ConstSharedPtr = const SharedPtr;

  explicit TransformProvider(const std::string & path_file_ascii_output);

  void process();

  struct Velocity
  {
    double x{0U};
    double y{0U};
    double z{0U};
  };
  struct Pose
  {
    uint32_t stamp_unix_seconds{0U};
    uint32_t stamp_nanoseconds{0U};
    Velocity velocity;
    geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
    float meridian_convergence;
  };
  struct Imu
  {
    uint32_t stamp_unix_seconds{0U};
    uint32_t stamp_nanoseconds{0U};
    sensor_msgs::msg::Imu imu;
  };

  std::vector<Pose> poses_;
  std::vector<Imu> imu_rotations_;

  Pose get_pose_at(
    uint32_t stamp_unix_seconds,
    uint32_t stamp_nanoseconds);

  Imu get_imu_at(uint32_t stamp_unix_seconds, uint32_t stamp_nanoseconds);

  std::vector<double> parse_mgrs_coordinates(const std::string & mgrs_string);
  std::string parse_mgrs_zone(const std::string & mgrs_string);


private:
  fs::path path_file_ascii_output_;
  std::string header_line_string;
  std::string time_string;
  int data_line_number;
  std::string mission_date;
  double last_utc_time = 0.0;
  bool day_changed_flag = false;

  float compute_meridian_convergence(double lat, double lon);

  int last_index_imu;
  int last_index_pose;
};
}  // mapora::transform_provider

#endif  // MAPORA__TRANSFORM_PROVIDER_APPLANIX_HPP_
