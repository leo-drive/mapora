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

#ifndef MAPORA__CONTINUOUS_PACKET_PARSER_HPP_
#define MAPORA__CONTINUOUS_PACKET_PARSER_HPP_

#include <map>
#include <memory>
#include <vector>
#include <cstdint>
#include <string>
#include <functional>
#include "point_xyzit.hpp"
#include "mapora/date.h"
#include <pcapplusplus/Packet.h>


namespace mapora::points_provider::continuous_packet_parser_vlp16
{
class ContinuousPacketParserVlp16
{
public:
  using Point = point_types::PointXYZIT;
  using Points = std::vector<Point>;

  ContinuousPacketParserVlp16();

  void process_packet_into_cloud(
    const pcpp::RawPacket & rawPacket,
    const std::function<void(const Points &)> & callback_cloud_surround_out,
    const float min_point_distance_from_lidar,
    const float max_point_distance_from_lidar);

private:
  using uint8_t = std::uint8_t;
  using uint16_t = std::uint16_t;

  struct DataPoint
  {
    uint16_t distance_divided_by_2mm;
    uint8_t reflectivity;
  } __attribute__((packed));

  struct DataBlock
  {
    uint8_t flag_1;
    uint8_t flag_2;
    uint16_t azimuth_multiplied_by_100_deg;
    DataPoint data_points[32];
    [[nodiscard]] size_t get_size_data_points() const
    {
      return sizeof(data_points) / sizeof(data_points[0]);
    }
  } __attribute__((packed));

  struct DataPacket
  {
    uint8_t udp_header[42];
    DataBlock data_blocks[12];
    uint32_t microseconds_toh;
    uint8_t factory_byte_return_mode;
    uint8_t factory_byte_product_id;
    [[nodiscard]] size_t get_size_data_blocks() const
    {
      return sizeof(data_blocks) / sizeof(data_blocks[0]);
    }
  } __attribute__((packed));

  enum class ReturnMode { Strongest, LastReturn, DualReturn, DualReturnWithConfidence };

  std::map<uint8_t, ReturnMode> map_byte_to_return_mode_;
  std::map<ReturnMode, std::string> map_return_mode_to_string_;

  enum class VelodyneModel
  {
    HDL32E,
    VLP16orPuckLITE,
    PuckHiRes,
    VLP32CorVLP32MR,
    Velarray,
    VLS128
  };


  struct PositionPacket
  {
    uint8_t udp_header[42];
    uint8_t reserved_01[187];
    uint8_t temp_top_board;
    uint8_t temp_bot_board;
    uint8_t reserved_02[9];
    uint32_t timestamp_microseconds_since_hour;
    uint8_t status_pps;
    uint8_t status_thermal;
    uint8_t temp_when_last_shut_from_overheat;
    uint8_t temp_when_booted;
    char nmea_sentence[128];
    uint8_t reserved_03[178];
  } __attribute__((packed));


  VelodyneModel velodyne_model_;
  ReturnMode return_mode_;

  bool factory_bytes_are_read_at_least_once_;
  bool has_received_valid_position_package_;

  date::sys_time<std::chrono::hours> tp_hours_since_epoch;

  std::map<uint8_t, VelodyneModel> map_byte_to_velodyne_model_;
  std::map<VelodyneModel, std::string> map_velodyne_model_to_string_;

  std::vector<float> channel_to_angle_vertical_;
  std::vector<float> channel_mod_8_to_azimuth_offsets_;
  std::vector<size_t> ind_block_to_first_channel_;

  bool has_processed_a_packet_;
  float angle_deg_azimuth_last_packet_;
  uint32_t microseconds_last_packet_;

  Points cloud_;
  bool can_publish_again_;
  float angle_deg_cut_;
};
}  // namespace mapora::point_provider::continuous_packet_parser_vlp16


#endif  // MAPORA__CONTINUOUS_PACKET_PARSER_HPP_
