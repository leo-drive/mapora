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
#include "point_types.hpp"
#include "mapora/date.h"
#include "mapora/parsers/velodyne_vlp16.hpp"

namespace mapora::points_provider
{
namespace fs = boost::filesystem;
class PointsProvider
{
public:
  using SharedPtr = std::shared_ptr<PointsProvider>;
  using ConstSharedPtr = const SharedPtr;
  using Point = point_types::PointXYZITRH;
  using Points = std::vector<Point>;

  explicit PointsProvider(
      std::string  path_folder_pcaps, std::string sensor_type);

  void process();

  void process_pcaps_into_clouds(
    std::function<void(const Points &)> & callback_cloud_surround_out,
    size_t index_start,
    size_t count,
    float max_point_distance_from_lidar,
    float min_point_distance_from_lidar);
  std::string info();

  std::vector<fs::path> paths_pcaps_;


  template <typename parser_type>
  void process_pcap_into_clouds(
    const fs::path & path_pcap,
    const std::function<void(const Points &)>& callback_cloud_surround_out,
//      continuous_packet_parser::ContinuousPacketParserVlp16& parser,
      parser_type &parser,
    const float min_point_distance_from_lidar,
    const float max_point_distance_from_lidar);
private:
  fs::path path_folder_pcaps_;
  std::string sensor_type_;


};
}  // namespace mapora::points_provider

#endif  // MAPORA__POINTS_PROVIDER_VELODYNE_VLP16_HPP_
