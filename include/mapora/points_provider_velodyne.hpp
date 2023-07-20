// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__POINTS_PROVIDER_VELODYNE_HPP_
#define MAPORA__POINTS_PROVIDER_VELODYNE_HPP_

#include <boost/filesystem.hpp>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <functional>
#include "points_provider_base.hpp"
#include "point_xyzit.hpp"
#include "mapora/date.h"
#include "mapora/continuous_packet_parser.hpp"

namespace mapora::points_provider
{
namespace fs = boost::filesystem;
class PointsProviderVelodyne : public virtual PointsProviderBase
{
public:
  using SharedPtr = std::shared_ptr<PointsProviderVelodyne>;
  using ConstSharedPtr = const SharedPtr;

  explicit PointsProviderVelodyne( std::string  path_folder_pcaps);

  void process() override;

  void process_pcaps_into_clouds(
    std::function<void(const Points &)> & callback_cloud_surround_out,
    size_t index_start,
    size_t count,
    size_t count_clouds_to_extract_max = 0);
  std::string info() override;

  void process_pcap_into_clouds(
    const fs::path & path_pcap,
    const std::function<void(const Points &)>& callback_cloud_surround_out,
    continuous_packet_parser::ContinuousPacketParser& parser);

private:
  fs::path path_folder_pcaps_;

  std::vector<fs::path> paths_pcaps_;
};
}  // namespace mapora::points_provider

#endif  // MAPORA__POINTS_PROVIDER_VELODYNE_HPP_
