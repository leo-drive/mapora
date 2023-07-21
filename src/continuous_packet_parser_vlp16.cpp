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

#include "mapora/continuous_packet_parser_vlp16.hpp"
#include <iostream>
#include <pcapplusplus/Packet.h>
#include "mapora/utils.hpp"

namespace mapora::points_provider::continuous_packet_parser_vlp16 {
ContinuousPacketParserVlp16::ContinuousPacketParserVlp16()
  : factory_bytes_are_read_at_least_once_{false},
    has_received_valid_position_package_{false},
    has_processed_a_packet_{false},
    angle_deg_azimuth_last_packet_{0.0f},
    microseconds_last_packet_{0U},
    can_publish_again_{true},
    angle_deg_cut_{90.0f} {
  map_byte_to_return_mode_.insert(std::make_pair(55, ReturnMode::Strongest));
  map_byte_to_return_mode_.insert(std::make_pair(56, ReturnMode::LastReturn));
  map_byte_to_return_mode_.insert(std::make_pair(57, ReturnMode::DualReturn));
  map_byte_to_return_mode_.insert(std::make_pair(59, ReturnMode::DualReturnWithConfidence));

  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::Strongest, "Strongest"));
  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::LastReturn, "LastReturn"));
  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::DualReturn, "DualReturn"));
  map_return_mode_to_string_.insert(
    std::make_pair(ReturnMode::DualReturnWithConfidence, "DualReturnWithConfidence"));

  map_byte_to_velodyne_model_.insert(std::make_pair(33, VelodyneModel::HDL32E));
  map_byte_to_velodyne_model_.insert(std::make_pair(34, VelodyneModel::VLP16orPuckLITE));
  map_byte_to_velodyne_model_.insert(std::make_pair(36, VelodyneModel::PuckHiRes));
  map_byte_to_velodyne_model_.insert(std::make_pair(40, VelodyneModel::VLP32CorVLP32MR));
  map_byte_to_velodyne_model_.insert(std::make_pair(49, VelodyneModel::Velarray));
  map_byte_to_velodyne_model_.insert(std::make_pair(161, VelodyneModel::VLS128));

  map_velodyne_model_to_string_.insert(std::make_pair(VelodyneModel::HDL32E, "HDL32E"));
  map_velodyne_model_to_string_.insert(
    std::make_pair(VelodyneModel::VLP16orPuckLITE, "VLP16orPuckLITE"));
  map_velodyne_model_to_string_.insert(std::make_pair(VelodyneModel::PuckHiRes, "PuckHiRes"));
  map_velodyne_model_to_string_.insert(
    std::make_pair(VelodyneModel::VLP32CorVLP32MR, "VLP32CorVLP32MR"));
  map_velodyne_model_to_string_.insert(std::make_pair(VelodyneModel::Velarray, "Velarray"));
  map_velodyne_model_to_string_.insert(std::make_pair(VelodyneModel::VLS128, "VLS128"));

  channel_to_angle_vertical_ = std::vector<float>{
    -15.0F, 1.0F, -13.0F, 3.0F, -11.0F, 5.0F, -9.0F, 7.0F, -7.0F, 9.0F, -5.0F, 11.0F, -3.0F,
    13.0F, -1.0F, 15.0F,
    -15.0F, 1.0F, -13.0F, 3.0F, -11.0F, 5.0F, -9.0F, 7.0F, -7.0F, 9.0F, -5.0F, 11.0F, -3.0F,
    13.0F, -1.0F, 15.0F};
  assert(channel_to_angle_vertical_.size() == 32);  //  VLP-16 has 32 channels in each data block

  channel_mod_8_to_azimuth_offsets_ =
    std::vector<float>{-6.354F, -4.548F, -2.732F, -0.911F, 0.911F, 2.732F, 4.548F, 6.354F};

  std::vector<size_t> vec_counting_numbers(12);
  std::iota(vec_counting_numbers.begin(), vec_counting_numbers.end(), 0);
  ind_block_to_first_channel_.resize(12);
  std::fill(ind_block_to_first_channel_.begin(), ind_block_to_first_channel_.end(), 0);
}


void ContinuousPacketParserVlp16::process_packet_into_cloud(
  const pcpp::RawPacket &rawPacket,
  const std::function<void(const Points &)> &callback_cloud_surround_out,
  const float min_point_distance_from_lidar,
  const float max_point_distance_from_lidar) {
  switch (rawPacket.getFrameLength()) {
    case 554: {
      // Position Packet
      if (has_received_valid_position_package_) { break; }

      auto *position_packet = reinterpret_cast<const PositionPacket *>(rawPacket.getRawData());

      //        https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm


      std::string nmea_sentence(position_packet->nmea_sentence);
      auto segments_with_nullstuff = utils::Utils::string_to_vec_split_by(nmea_sentence, '\r');

      auto segments_with_crc =
        utils::Utils::string_to_vec_split_by(segments_with_nullstuff.front(), '*');
      auto segments = utils::Utils::string_to_vec_split_by(segments_with_crc.front(), ',');
      if ( 13 > segments.size() || 14 < segments.size()) {
        throw std::length_error(
          "nmea sentence should have 13 elements, it has " +
          std::to_string(segments.size()));
      }

      // Receiver status: A= Active, V= Void
      if (segments.at(2) != "A") {
        std::cout << "Receiver Status != Active" << std::endl;
        break;
      }

      const auto &str_time = segments.at(1);
      int hours_raw = std::stoi(str_time.substr(0, 2));

      const auto &str_date = segments.at(9);
      int days_raw = std::stoi(str_date.substr(0, 2));
      int months_raw = std::stoi(str_date.substr(2, 2));
      int years_raw = 2000 + std::stoi(str_date.substr(4, 2));

      date::year_month_day date_current_ = date::year{years_raw} / months_raw / days_raw;
      tp_hours_since_epoch = date::sys_days(date_current_) + std::chrono::hours(hours_raw);
      uint32_t seconds_epoch_precision_of_hour =
        std::chrono::seconds(tp_hours_since_epoch.time_since_epoch()).count();

      has_received_valid_position_package_ = true;
      break;
    }
    case 1248: {
      // Data Packet
      if (!has_received_valid_position_package_) {
        // Ignore until first valid Position Packet is received
        break;
      }
      const auto *data_packet_with_header =
        reinterpret_cast<const DataPacket *>(rawPacket.getRawData());

      // TOH = Top Of the Hour
      date::hh_mm_ss microseconds_since_toh =
        date::make_time(std::chrono::microseconds(data_packet_with_header->microseconds_toh));
      //          std::cout << "data_packet_with_header->timestamp_microseconds_since_hour: " <<
      //            data_packet_with_header->timestamp_microseconds_since_hour << std::endl;
      //          std::cout << "date_time.hours(): " << microseconds_since_toh.hours().count() <<
      //          std::endl; std::cout << "date_time.minutes(): " <<
      //          microseconds_since_toh.minutes().count() << std::endl; std::cout <<
      //          "date_time.seconds(): " << microseconds_since_toh.seconds().count() <<
      //          std::endl;

      auto velodyne_model =
        map_byte_to_velodyne_model_.at(data_packet_with_header->factory_byte_product_id);
      auto return_mode =
        map_byte_to_return_mode_.at(data_packet_with_header->factory_byte_return_mode);

      // I can only process Single Return Modes for now
      if (return_mode != ReturnMode::Strongest && return_mode != ReturnMode::LastReturn) {
        throw std::runtime_error(
          "return_mode was expected to be either: " +
          map_return_mode_to_string_.at(ReturnMode::Strongest) +
          " or: " + map_return_mode_to_string_.at(ReturnMode::LastReturn) +
          " but it was: " + map_return_mode_to_string_.at(return_mode));
      }

      // I can only process VLS128 data packets for now
      if (velodyne_model != VelodyneModel::VLP16orPuckLITE) {
        throw std::runtime_error(
          "velodyne_model was expected to be: " +
          map_velodyne_model_to_string_.at(VelodyneModel::VLP16orPuckLITE) +
          " but it was: " + map_velodyne_model_to_string_.at(velodyne_model));
      }

      // Once factory bytes are received, it is expected to be the same in every data packet.
      if (!factory_bytes_are_read_at_least_once_) {
        velodyne_model_ = velodyne_model;
        return_mode_ = return_mode;
        factory_bytes_are_read_at_least_once_ = true;
      } else {
        if (velodyne_model != velodyne_model_) {
          throw std::runtime_error(
            "velodyne_model was expected to be: " +
            map_velodyne_model_to_string_.at(velodyne_model_) +
            " but it was: " + map_velodyne_model_to_string_.at(velodyne_model));
        }
        if (return_mode != return_mode_) {
          throw std::runtime_error(
            "return_mode was expected to be: " +
            map_return_mode_to_string_.at(return_mode_) +
            " but it was: " + map_return_mode_to_string_.at(return_mode));
        }
      }

      // Iterate through 12 blocks
      double speed_deg_per_microseconds_angle_azimuth;
      float angle_deg_azimuth_last;
      for (size_t ind_block = 0; ind_block < data_packet_with_header->get_size_data_blocks();
           ind_block++) {
        const auto &data_block = data_packet_with_header->data_blocks[ind_block];
        float angle_deg_azimuth_of_block =
          static_cast<float>(data_block.azimuth_multiplied_by_100_deg) / 100.0f;

        if (!has_processed_a_packet_) {
          angle_deg_azimuth_last_packet_ = angle_deg_azimuth_of_block;
          microseconds_last_packet_ = data_packet_with_header->microseconds_toh;
          has_processed_a_packet_ = true;
          break;
        } else if (ind_block == 0) {
          // Compensate for azimuth angular rollover
          float angle_deg_azimuth_increased = angle_deg_azimuth_of_block;
          if (angle_deg_azimuth_of_block < angle_deg_azimuth_last_packet_) {
            angle_deg_azimuth_increased += 360.0f;
          }
          float angle_deg_angle_delta =
            angle_deg_azimuth_increased - angle_deg_azimuth_last_packet_;

          // Compensate for ToH microseconds rollover

          uint32_t microseconds_toh_current_increased =
            data_packet_with_header->microseconds_toh;
          if (data_packet_with_header->microseconds_toh < microseconds_last_packet_) {
            microseconds_toh_current_increased += 3600000000U;
            // Increase internal epoch hour time point
            tp_hours_since_epoch += std::chrono::hours(1);
          }
          uint32_t microseconds_delta =
            microseconds_toh_current_increased - microseconds_last_packet_;

          speed_deg_per_microseconds_angle_azimuth =
            static_cast<double>(angle_deg_angle_delta) / microseconds_delta;

          angle_deg_azimuth_last_packet_ = angle_deg_azimuth_of_block;
          microseconds_last_packet_ = data_packet_with_header->microseconds_toh;
        }


        // Iterate through 32 points within a block
        for (size_t ind_point = 0; ind_point < data_block.get_size_data_points(); ind_point++) {
          const auto &data_point = data_block.data_points[ind_point];

          double timing_offset_from_first_firing;
          if (ind_point > 15){
            timing_offset_from_first_firing =
              18.432 + 2.304 * static_cast<int>(ind_point);
          } else {
            timing_offset_from_first_firing =
              2.304 * static_cast<int>(ind_point);
          }
          float angle_deg_azimuth_point =
            angle_deg_azimuth_of_block +
            static_cast<float>(
              speed_deg_per_microseconds_angle_azimuth * timing_offset_from_first_firing);

          if (angle_deg_azimuth_point >= 360.0f) { angle_deg_azimuth_point -= 360.0f; }

          angle_deg_azimuth_last = angle_deg_azimuth_point;
          float angle_rad_azimuth_point = utils::Utils::deg_to_rad(angle_deg_azimuth_point);

          float angle_deg_vertical = channel_to_angle_vertical_.at(ind_point);
          float angle_rad_vertical = utils::Utils::deg_to_rad(angle_deg_vertical);
          float dist_m = static_cast<float>(data_point.distance_divided_by_2mm * 2) / 1000.0f;
          float dist_xy = dist_m * std::cos(angle_rad_vertical);
          Point point;
          point.x = dist_xy * std::sin(angle_rad_azimuth_point);
          point.y = dist_xy * std::cos(angle_rad_azimuth_point);
          point.z = dist_m * std::sin(angle_rad_vertical);
          point.intensity = data_point.reflectivity;
          point.stamp_unix_seconds =
            std::chrono::seconds(
              tp_hours_since_epoch.time_since_epoch() + microseconds_since_toh.minutes() +
              microseconds_since_toh.seconds())
              .count();
          point.stamp_nanoseconds =
            std::chrono::nanoseconds(microseconds_since_toh.subseconds()).count();

          if (
            std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2))
            < min_point_distance_from_lidar) {
            continue;
          }
          if (
            std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2))
            > max_point_distance_from_lidar) {
            continue;
          }

          cloud_.push_back(point);
        }
      }

      bool is_close_to_cut_area = std::abs(angle_deg_azimuth_last - angle_deg_cut_) < 5.0f;

      if (!is_close_to_cut_area) {
        can_publish_again_ = true;
      }

      if (can_publish_again_ && is_close_to_cut_area) {
        callback_cloud_surround_out(cloud_);
        cloud_.clear();
        can_publish_again_ = false;
      }

      break;
    }
    default: {
      //          std::cerr << "Unknown package with length: " << rawPacket.getFrameLength() <<
      //          std::endl;
      break;
    }
  }
}
}  // namespace mapora::point_provider::continuous_packet_parser_vlp16
