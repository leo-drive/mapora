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

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "mapora/parsers/hesai_xt32.hpp"
#include "mapora/parsers/velodyne_vlp16.hpp"
#include "mapora/point_types.hpp"
#include "mapora/points_provider.hpp"
#include <algorithm>
#include <exception>
#include <iostream>
#include <pcapplusplus/PcapFileDevice.h>
#include <string>
#include <vector>

namespace mapora::points_provider {
namespace fs = boost::filesystem;
using Point = point_types::PointXYZITRH;
using Points = std::vector<Point>;

PointsProvider::PointsProvider(
    std::string path_folder_pcaps, std::string sensor_type)
  : path_folder_pcaps_{path_folder_pcaps} {
  if (!fs::is_directory(path_folder_pcaps_)) {
    throw std::runtime_error(path_folder_pcaps_.string() + " is not a directory.");
  }
  sensor_type_ = sensor_type;
}

void PointsProvider::process() {
  paths_pcaps_.clear();
  for (const auto &path_pcap :
    boost::make_iterator_range(fs::directory_iterator(path_folder_pcaps_))) {
    if (fs::is_directory(path_pcap.path())) { continue; }
    if (path_pcap.path().extension() != ".pcap") { continue; }
//    std::cout << "pcap: " << path_pcap.path().string() << std::endl;
    paths_pcaps_.push_back(path_pcap);
  }

  // sort paths
  std::sort(paths_pcaps_.begin(), paths_pcaps_.end(),
            [](const fs::path &a, const fs::path &b) {
              return a.filename().string() < b.filename().string();
            });

  if (paths_pcaps_.empty()) {
    throw std::runtime_error(path_folder_pcaps_.string() + " doesn't contain a pcap file.");
  }
}

void PointsProvider::process_pcaps_into_clouds(
  std::function<void(const Points &)> &callback_cloud_surround_out,
  const size_t index_start,
  const size_t count,
  const float min_point_distance_from_lidar,
  const float max_point_distance_from_lidar) {
  if (index_start >= paths_pcaps_.size() || index_start + count > paths_pcaps_.size()) {
    throw std::range_error("index is outside paths_pcaps_ range.");
  }

  if (sensor_type_ == "velodyne_vlp16") {
    continuous_packet_parser::ContinuousPacketParserVlp16 packet_parser;
    for (size_t i = index_start; i < index_start + count; ++i) {
      process_pcap_into_clouds(paths_pcaps_.at(i), callback_cloud_surround_out,
                               packet_parser, min_point_distance_from_lidar, max_point_distance_from_lidar);
    }
  } else if (sensor_type_ == "hesai_xt32") {
    continuous_packet_parser::ContinuousPacketParserXt32 packet_parser;
    for (size_t i = index_start; i < index_start + count; ++i) {
      process_pcap_into_clouds(paths_pcaps_.at(i), callback_cloud_surround_out,
                               packet_parser, min_point_distance_from_lidar, max_point_distance_from_lidar);
    }
  }
}

template <typename parser_type>
void PointsProvider::process_pcap_into_clouds(
  const fs::path &path_pcap,
  const std::function<void(const Points &)> &callback_cloud_surround_out,
//    continuous_packet_parser::ContinuousPacketParserVlp16 &parser,
    parser_type &parser,
  const float min_point_distance_from_lidar,
  const float max_point_distance_from_lidar) {
//  std::cout << "processing: " << path_pcap << std::endl;
  pcpp::IFileReaderDevice *reader = pcpp::IFileReaderDevice::getReader(path_pcap.string());
  if (reader == nullptr) {
    printf("Cannot determine reader for file type\n");
    exit(1);
  }

  if (!reader->open()) {
    printf("Cannot open input.pcap for reading\n");
    exit(1);
  }

  pcpp::RawPacket rawPacket;
  while (reader->getNextPacket(rawPacket)) {
    parser.process_packet_into_cloud(rawPacket, callback_cloud_surround_out,
                                     min_point_distance_from_lidar, max_point_distance_from_lidar);
  }

  reader->close();
  delete reader;
}

std::string PointsProvider::info() { return ""; }
}  // namespace mapora::points_provider
