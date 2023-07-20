// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <pcapplusplus/PcapFileDevice.h>
#include <exception>
#include <utility>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include "mapora/point_xyzit.hpp"
#include "mapora/utils.hpp"
#include "mapora/date.h"
#include "mapora/points_provider_velodyne.hpp"
#include "mapora/continuous_packet_parser.hpp"

namespace mapora::points_provider
{
namespace fs = boost::filesystem;
using Point = PointsProviderBase::Point;
using Points = PointsProviderBase::Points;

PointsProviderVelodyne::PointsProviderVelodyne(std::string path_folder_pcaps)
: path_folder_pcaps_{path_folder_pcaps}
{
  if (!fs::is_directory(path_folder_pcaps_)) {
    throw std::runtime_error(path_folder_pcaps_.string() + " is not a directory.");
  }
}

void PointsProviderVelodyne::process()
{
  paths_pcaps_.clear();
  for (const auto & path_pcap :
    boost::make_iterator_range(fs::directory_iterator(path_folder_pcaps_)))
  {
    if (fs::is_directory(path_pcap.path())) {continue;}
    if (path_pcap.path().extension() != ".pcap") {continue;}
    //    std::cout << "pcap: " << path_pcap.path().string() << std::endl;
    paths_pcaps_.push_back(path_pcap);
  }
  std::sort(paths_pcaps_.begin(), paths_pcaps_.end());

  if (paths_pcaps_.empty()) {
    throw std::runtime_error(path_folder_pcaps_.string() + " doesn't contain a pcap file.");
  }
}

void PointsProviderVelodyne::process_pcaps_into_clouds(
  std::function<void(const Points &)> & callback_cloud_surround_out,
  const size_t index_start,
  const size_t count,
  const size_t count_clouds_to_extract_max)
{
  if (index_start >= paths_pcaps_.size() || index_start + count > paths_pcaps_.size()) {
    throw std::range_error("index is outside paths_pcaps_ range.");
  }

  continuous_packet_parser::ContinuousPacketParser packet_parser(count_clouds_to_extract_max);
  for (size_t i = index_start; i < index_start + count; ++i) {
    process_pcap_into_clouds(paths_pcaps_.at(i), callback_cloud_surround_out, packet_parser);
  }
}

void PointsProviderVelodyne::process_pcap_into_clouds(
  const fs::path & path_pcap,
  const std::function<void(const Points &)> & callback_cloud_surround_out,
  continuous_packet_parser::ContinuousPacketParser & parser)
{
  std::cout << "processing: " << path_pcap << std::endl;
  pcpp::IFileReaderDevice * reader = pcpp::IFileReaderDevice::getReader(path_pcap.string());
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
    parser.process_packet_into_cloud(rawPacket, callback_cloud_surround_out);
  }

  reader->close();
  delete reader;
}


std::string PointsProviderVelodyne::info() {return "";}
}  // namespace mapora::points_provider
