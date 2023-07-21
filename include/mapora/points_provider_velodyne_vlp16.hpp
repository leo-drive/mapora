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

#ifndef MAPORA__POINTS_PROVIDER_VELODYNE_VLP16_HPP_
#define MAPORA__POINTS_PROVIDER_VELODYNE_VLP16_HPP_

#include <boost/filesystem.hpp>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <functional>
#include "points_provider_base.hpp"
#include "point_xyzit.hpp"
#include "mapora/date.h"
#include "mapora/continuous_packet_parser_vlp16.hpp"

namespace mapora::points_provider
{
namespace fs = boost::filesystem;
class PointsProviderVelodyneVlp16 : public virtual PointsProviderBase
{
public:
  using SharedPtr = std::shared_ptr<PointsProviderVelodyneVlp16>;
  using ConstSharedPtr = const SharedPtr;

  explicit PointsProviderVelodyneVlp16( std::string  path_folder_pcaps);

  void process() override;

  void process_pcaps_into_clouds(
    std::function<void(const Points &)> & callback_cloud_surround_out,
    size_t index_start,
    size_t count,
    float max_point_distance_from_lidar,
    float min_point_distance_from_lidar);
  std::string info() override;

  std::vector<fs::path> paths_pcaps_;

  void process_pcap_into_clouds(
    const fs::path & path_pcap,
    const std::function<void(const Points &)>& callback_cloud_surround_out,
    continuous_packet_parser_vlp16::ContinuousPacketParserVlp16& parser,
    const float min_point_distance_from_lidar,
    const float max_point_distance_from_lidar);
private:
  fs::path path_folder_pcaps_;
};
}  // namespace mapora::points_provider

#endif  // MAPORA__POINTS_PROVIDER_VELODYNE_VLP16_HPP_
