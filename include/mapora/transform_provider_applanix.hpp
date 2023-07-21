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

namespace mapora::transform_provider_applanix
{
namespace fs = boost::filesystem;
class TransformProviderApplanix
{
public:
  using SharedPtr = std::shared_ptr<TransformProviderApplanix>;
  using ConstSharedPtr = const SharedPtr;

  explicit TransformProviderApplanix(const std::string & path_file_ascii_output);

  void process();

  struct Pose
  {
    uint32_t stamp_unix_seconds{0U};
    uint32_t stamp_nanoseconds{0U};
    geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  };

  std::vector<Pose> poses_;
  std::string header_line_string;
  std::string time_string;
  int data_line_number;
  std::string mission_date;

  Pose get_pose_at(
    uint32_t stamp_unix_seconds,
    uint32_t stamp_nanoseconds);

private:
  fs::path path_file_ascii_output_;
};
}  // mapora::transform_provider_applanix

#endif  // MAPORA__TRANSFORM_PROVIDER_APPLANIX_HPP_
