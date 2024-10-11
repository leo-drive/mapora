//
// Created by ataparlar on 11.10.2024.
//

#ifndef MAPORA__CONTINUOUS_PACKET_PARSER_HESAI_XT32_HPP_
#define MAPORA__CONTINUOUS_PACKET_PARSER_HESAI_XT32_HPP_


#include "mapora/date.h"
#include "mapora/point_types.hpp"

#include <pcapplusplus/Packet.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mapora::points_provider::continuous_packet_parser
{
class ContinuousPacketParserXt32 {
public:
  using SharedPtr = std::shared_ptr<ContinuousPacketParserXt32>;
  using ConstSharedPtr = const std::shared_ptr<ContinuousPacketParserXt32>;
  using Point = point_types::PointXYZITRH;
  using Points = std::vector<Point>;

  ContinuousPacketParserXt32();

  void process_packet_into_cloud(
      const pcpp::RawPacket & rawPacket,
      const std::function<void(const Points &)> & callback_cloud_surround_out,
      double time_start_in_utc, double time_end_in_utc,
      const float min_point_distance_from_lidar,
      const float max_point_distance_from_lidar);

private:
  using uint8_t = std::uint8_t;
  using uint16_t = std::uint16_t;

  struct PreHeader
  {
    uint8_t start_of_packet_1;
    uint8_t start_of_packet_2;
    uint8_t protocol_version_major;
    uint8_t protocol_version_minor;
    uint16_t reserved;
  } __attribute__((packed));

  struct Header
  {
    uint8_t laser_number;
    uint8_t block_number;
    uint8_t first_block_return;
    uint8_t dis_unit;
    uint8_t return_number;
    uint8_t udp_seq;
  } __attribute__((packed));

  struct DataPoint
  {
    uint16_t distance_divided_by_4mm;
    uint8_t reflectivity;
    uint8_t reserved;
  } __attribute__((packed));

  struct DataBlock  // each 130 byte
  {
    uint16_t azimuth_multiplied_by_100_deg;
    DataPoint data_points[32];
    [[nodiscard]] size_t get_size_data_points() const
    {
      return sizeof(data_points) / sizeof(data_points[0]);
    }
  } __attribute__((packed));

  struct Tail
  {
    uint8_t reserved[9];
    uint8_t high_temp_shutdown;
    uint8_t return_mode;
    uint16_t motor_speed;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t timestamp;
    uint8_t factory_info;
  } __attribute__((packed));

  struct DataPacket {
    uint8_t udp_header[42];
    PreHeader pre_header;
    Header header;
    DataBlock data_blocks[8];
    Tail tail;
    uint32_t udp_seq;;
    size_t get_size_data_blocks() const
    {
      return sizeof(data_blocks) / sizeof(data_blocks[0]);
    }
  };


  enum class ReturnMode { Strongest, LastReturn, DualReturn };

  std::map<uint8_t, ReturnMode> map_byte_to_return_mode_;
  std::map<ReturnMode, std::string> map_return_mode_to_string_;

  enum class HesaiModel
  {
    Pandar128,  // 1
    ET,         // 2
    QT,         // 3
    AT128,      // 4
    PandarXT,   // 6
    PandarFT    // 7
  };


  struct PositionPacket
  {
    uint8_t udp_header[42];
    uint16_t header;
    uint8_t date[6];
    uint8_t time[6];
    uint32_t microsecond_time;
    char nmea_sentence[84];
    uint8_t reserved[404];
    uint8_t gps_positioning_status;
    uint8_t status_pps;
    uint32_t reserved_2;
    //    uint8_t temp_when_last_shut_from_overheat;
    //    uint8_t temp_when_booted;
    //    char nmea_sentence[128];
    //    uint8_t reserved_03[178];
  } __attribute__((packed));


  HesaiModel hesai_model_;
  ReturnMode return_mode_;

  bool factory_bytes_are_read_at_least_once_;
  bool has_received_valid_position_package_;

  date::sys_time<std::chrono::hours> tp_hours_since_epoch;

  std::map<uint8_t, HesaiModel> map_byte_to_hesai_model_;
  std::map<HesaiModel, std::string> map_hesai_model_to_string_;

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
}



#endif // MAPORA__CONTINUOUS_PACKET_PARSER_HESAI_XT32_HPP_
