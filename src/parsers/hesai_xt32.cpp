//
// Created by ataparlar on 11.10.2024.
//

#include "mapora/parsers/hesai_xt32.hpp"
#include "mapora/utils.hpp"

#include <iostream>


namespace mapora::points_provider::continuous_packet_parser
{

ContinuousPacketParserXt32::ContinuousPacketParserXt32()
    : factory_bytes_are_read_at_least_once_{false},
      has_received_valid_position_package_{false},
      has_processed_a_packet_{false},
      angle_deg_azimuth_last_packet_{0.0f},
      microseconds_last_packet_{0U},
      can_publish_again_{true},
      angle_deg_cut_{90.0f}
{
  map_byte_to_return_mode_.insert(std::make_pair(55, ReturnMode::Strongest));
  map_byte_to_return_mode_.insert(std::make_pair(56, ReturnMode::LastReturn));
  map_byte_to_return_mode_.insert(std::make_pair(57, ReturnMode::DualReturn));
  //  map_byte_to_return_mode_.insert(std::make_pair(59, ReturnMode::DualReturnWithConfidence));

  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::Strongest, "Strongest"));
  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::LastReturn, "LastReturn"));
  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::DualReturn, "DualReturn"));
  //  map_return_mode_to_string_.insert(
  //    std::make_pair(ReturnMode::DualReturnWithConfidence, "DualReturnWithConfidence"));

  map_byte_to_hesai_model_.insert(std::make_pair(1, HesaiModel::Pandar128));
  map_byte_to_hesai_model_.insert(std::make_pair(2, HesaiModel::ET));
  map_byte_to_hesai_model_.insert(std::make_pair(3, HesaiModel::QT));
  map_byte_to_hesai_model_.insert(std::make_pair(4, HesaiModel::AT128));
  map_byte_to_hesai_model_.insert(std::make_pair(6, HesaiModel::PandarXT));
  map_byte_to_hesai_model_.insert(std::make_pair(7, HesaiModel::PandarFT));

  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::Pandar128, "Pandar128"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::ET, "ET"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::QT, "QT"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::AT128, "AT128"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::PandarXT, "PandarXT"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::PandarFT, "PandarFT"));

  channel_to_angle_vertical_ = std::vector<float>{
      15.0F, 14.0F, 13.0F, 12.0F,  11.0F,  10.0F,  9.0F,   8.0F,   7.0F,   6.0F,  5.0F,
      4.0F,  3.0F,  2.0F,  1.0F,   0.0F,   -1.0F,  -2.0F,  -3.0F,  -4.0F,  -5.0F, -6.0F,
      -7.0F, -8.0F, -9.0F, -10.0F, -11.0F, -12.0F, -13.0F, -14.0F, -15.0F, -16.0F};
  assert(channel_to_angle_vertical_.size() == 32);
}

void ContinuousPacketParserXt32::process_packet_into_cloud(
    const pcpp::RawPacket & rawPacket,
    const std::function<void(const Points &)> & callback_cloud_surround_out,
    const float min_point_distance_from_lidar,
    const float max_point_distance_from_lidar)
{
  switch (rawPacket.getFrameLength()) {
  case 554: {
    if (has_received_valid_position_package_) {
      break;
    }

    auto * position_packet = reinterpret_cast<const PositionPacket *>(rawPacket.getRawData());

    std::string nmea_sentence(position_packet->nmea_sentence);
    auto segments_with_nullstuff = utils::Utils::string_to_vec_split_by(nmea_sentence, '\r');

    auto segments_with_crc =
        utils::Utils::string_to_vec_split_by(segments_with_nullstuff.front(), '*');
    auto segments = utils::Utils::string_to_vec_split_by(segments_with_crc.front(), ',');

    if (13 > segments.size() || 14 < segments.size()) {
      throw std::length_error(
          "nmea sentence should have 13 elements, it has " + std::to_string(segments.size()));
    }

    // Receiver status: A= Active, V= Void
    if (segments.at(2) != "A") {
      std::cout << "Receiver Status != Active" << std::endl;
      break;
    }
    const auto & str_time = segments.at(1);
    int hours_raw = std::stoi(str_time.substr(0, 2));

    const auto & str_date = segments.at(9);
    int days_raw = std::stoi(str_date.substr(0, 2));
    int months_raw = std::stoi(str_date.substr(2, 2));
    int years_raw = 2000 + std::stoi(str_date.substr(4, 2));

    date::year_month_day date_current_ = date::year{years_raw} / months_raw / days_raw;
    tp_hours_since_epoch = date::sys_days(date_current_) + std::chrono::hours(hours_raw);

//    for (auto segment : segments) {
//      std::cout << segment << std::endl;
//    }

    has_received_valid_position_package_ = true;
    break;
  }
  case 1122: {
    if (!has_received_valid_position_package_) {
      // Ignore until first valid Position Packet is received
      break;
    }

    const auto * data_packet_with_header =
        reinterpret_cast<const DataPacket *>(rawPacket.getRawData());

    date::hh_mm_ss microseconds_since_toh = date::make_time(
        //          std::chrono::hours(data_packet_with_header->tail.hour) +
        std::chrono::minutes(data_packet_with_header->tail.minute) +
        std::chrono::seconds(data_packet_with_header->tail.second) +
        std::chrono::microseconds(data_packet_with_header->tail.timestamp));
    double microseconds_toh = data_packet_with_header->tail.minute * 60000000 +
                              data_packet_with_header->tail.second * 1000000 +
                              data_packet_with_header->tail.timestamp;

    HesaiModel hesai_model;
    auto it = map_byte_to_hesai_model_.find(data_packet_with_header->pre_header.protocol_version_major);
    if (it == map_byte_to_hesai_model_.end()) {
      // Key not found, break the loop
      break;
    } else {
      hesai_model = it->second;
      // Proceed with your logic using hesai_model
    }

    if (hesai_model != HesaiModel::PandarXT) {
      throw std::runtime_error(
          "hesai_model was expected to be: " + map_hesai_model_to_string_.at(HesaiModel::PandarXT) +
          " but it was: " + map_hesai_model_to_string_.at(hesai_model));
    }


    ReturnMode return_mode;
    auto it2 = map_byte_to_return_mode_.find(data_packet_with_header->tail.return_mode);
    if (it2 == map_byte_to_return_mode_.end()) {
      // Key not found, break the loop
      break;
    } else {
      return_mode = it2->second;
      // Proceed with your logic using hesai_model
    }

    if (!factory_bytes_are_read_at_least_once_) {
      hesai_model_ = hesai_model;
      return_mode_ = return_mode;
      factory_bytes_are_read_at_least_once_ = true;
    } else {
      if (hesai_model != hesai_model_) {
        throw std::runtime_error(
            "hesai_model was expected to be: " + map_hesai_model_to_string_.at(hesai_model) +
            " but it was: " + map_hesai_model_to_string_.at(hesai_model));
      }
      if (return_mode != return_mode_) {
        throw std::runtime_error(
            "return_mode was expected to be: " + map_return_mode_to_string_.at(return_mode_) +
            " but it was: " + map_return_mode_to_string_.at(return_mode));
      }
    }

    double speed_deg_per_microseconds_angle_azimuth;
    float angle_deg_azimuth_last;

    if (return_mode == ReturnMode::DualReturn) {
      for (size_t ind_block = 0; ind_block < data_packet_with_header->get_size_data_blocks();
           ind_block += 2) {
        const auto & data_block_first = data_packet_with_header->data_blocks[ind_block];
        const auto & data_block_second = data_packet_with_header->data_blocks[ind_block + 1];

        float angle_deg_azimuth_of_block_first =
            static_cast<float>(data_block_first.azimuth_multiplied_by_100_deg) / 100.0f;
        float angle_deg_azimuth_of_block_second =
            static_cast<float>(data_block_second.azimuth_multiplied_by_100_deg) / 100.0f;
        if (angle_deg_azimuth_of_block_first != angle_deg_azimuth_of_block_second) continue;

        if (!has_processed_a_packet_) {
          angle_deg_azimuth_last_packet_ = angle_deg_azimuth_of_block_first;
          microseconds_last_packet_ = microseconds_toh;
          has_processed_a_packet_ = true;
          break;
        } else if (ind_block == 0) {
          float angle_deg_azimuth_increased = angle_deg_azimuth_of_block_first;
          if (angle_deg_azimuth_of_block_first < angle_deg_azimuth_last_packet_) {
            angle_deg_azimuth_increased += 360.0f;
          }
          float angle_deg_angle_delta =
              angle_deg_azimuth_increased - angle_deg_azimuth_last_packet_;

          // Compensate for ToH microseconds rollover
          uint32_t microseconds_toh_current_increased = microseconds_toh;
          if (microseconds_toh < microseconds_last_packet_) {
            microseconds_toh_current_increased += 3600000000U;
            // Increase internal epoch hour time point
            tp_hours_since_epoch += std::chrono::hours(1);
          }
          uint32_t microseconds_delta =
              microseconds_toh_current_increased - microseconds_last_packet_;

          speed_deg_per_microseconds_angle_azimuth =
              static_cast<double>(angle_deg_angle_delta) / microseconds_delta;

          angle_deg_azimuth_last_packet_ = angle_deg_azimuth_of_block_first;
          microseconds_last_packet_ = microseconds_toh;
        }

        for (int ind_point = 0; ind_point < data_block_first.get_size_data_points();
             ind_point++) {
          const auto & data_point_second = data_block_second.data_points[ind_point];

          float timing_offset_from_first_block;
          if (ind_block == 0 || ind_block == 1) {
            timing_offset_from_first_block = 3.28 - (50 * 3);
          } else if (ind_block == 2 || ind_block == 3) {
            timing_offset_from_first_block = 3.28 - (50 * 2);
          } else if (ind_block == 4 || ind_block == 5) {
            timing_offset_from_first_block = 3.28 - (50 * 1);
          } else if (ind_block == 6 || ind_block == 7) {
            timing_offset_from_first_block = 3.28;
          }
          float timing_offset_from_first_firing = 1.512 * (ind_point - 1) + 0.28;

          auto timing_offset_from_first_block_dur =
              std::chrono::duration<float, std::micro>(timing_offset_from_first_block);
          auto timing_offset_from_first_firing_dur =
              std::chrono::duration<float, std::micro>(timing_offset_from_first_firing);
          auto timing_offset_from_first_block_time =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  timing_offset_from_first_block_dur);
          auto timing_offset_from_first_firing_time =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  timing_offset_from_first_firing_dur);

          float angle_deg_azimuth_point =
              angle_deg_azimuth_of_block_first +
              static_cast<float>(
                  speed_deg_per_microseconds_angle_azimuth * timing_offset_from_first_firing);

          if (angle_deg_azimuth_point >= 360.0f) {
            angle_deg_azimuth_point -= 360.0f;
          }

          angle_deg_azimuth_last = angle_deg_azimuth_point;
          float angle_rad_azimuth_point = utils::Utils::deg_to_rad(angle_deg_azimuth_point);
          float angle_deg_vertical = channel_to_angle_vertical_.at(ind_point);
          float angle_rad_vertical = utils::Utils::deg_to_rad(angle_deg_vertical);
          float dist_m =
              static_cast<float>(data_point_second.distance_divided_by_4mm * 4) / 1000.0f;

          float dist_xy = dist_m * std::cos(angle_rad_vertical);
          Point point;
          point.x = dist_xy * std::sin(angle_rad_azimuth_point);
          point.y = dist_xy * std::cos(angle_rad_azimuth_point);
          point.z = dist_m * std::sin(angle_rad_vertical);
          point.intensity = data_point_second.reflectivity;
          point.ring = ind_point;
          point.horizontal_angle = angle_deg_azimuth_point;
          point.stamp_unix_seconds =
              std::chrono::seconds(
                  tp_hours_since_epoch.time_since_epoch() + microseconds_since_toh.minutes() +
                  microseconds_since_toh.seconds())
                  .count();
          point.stamp_nanoseconds =
              std::chrono::nanoseconds(
                  microseconds_since_toh.subseconds() + timing_offset_from_first_block_time +
                  timing_offset_from_first_firing_time)
                  .count();

          if (dist_m != 0 && dist_m <= max_point_distance_from_lidar &&
              dist_m >= min_point_distance_from_lidar) {
            cloud_.push_back(point);
          }
        }
      }
    } else if (return_mode == ReturnMode::LastReturn || return_mode == ReturnMode::Strongest) {
      for (size_t ind_block = 0; ind_block < data_packet_with_header->get_size_data_blocks();
           ind_block++) {
        const auto & data_block = data_packet_with_header->data_blocks[ind_block];
        float angle_deg_azimuth_of_block =
            static_cast<float>(data_block.azimuth_multiplied_by_100_deg) / 100.0f;

        if (!has_processed_a_packet_) {
          angle_deg_azimuth_last_packet_ = angle_deg_azimuth_of_block;
          microseconds_last_packet_ = microseconds_toh;
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

          uint32_t microseconds_toh_current_increased = microseconds_toh;
          if (microseconds_toh < microseconds_last_packet_) {
            microseconds_toh_current_increased += 3600000000U;
            // Increase internal epoch hour time point
            tp_hours_since_epoch += std::chrono::hours(1);
          }
          uint32_t microseconds_delta =
              microseconds_toh_current_increased - microseconds_last_packet_;

          speed_deg_per_microseconds_angle_azimuth =
              static_cast<double>(angle_deg_angle_delta) / microseconds_delta;

          angle_deg_azimuth_last_packet_ = angle_deg_azimuth_of_block;
          microseconds_last_packet_ = microseconds_toh;
        }

        for (size_t ind_point = 0; ind_point < data_block.get_size_data_points(); ind_point++) {
          const auto & data_point = data_block.data_points[ind_point];

          double timing_offset_from_first_block = 3.28 - (50 * (8 - ind_block + 1));
          double timing_offset_from_first_firing = 1.512 * (ind_point - 1) + 0.28;
          //            }

          auto timing_offset_from_first_block_dur =
              std::chrono::duration<float, std::micro>(timing_offset_from_first_block);
          auto timing_offset_from_first_firing_dur =
              std::chrono::duration<float, std::micro>(timing_offset_from_first_firing);
          auto timing_offset_from_first_block_time =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  timing_offset_from_first_block_dur);
          auto timing_offset_from_first_firing_time =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  timing_offset_from_first_firing_dur);

          float angle_deg_azimuth_point =
              angle_deg_azimuth_of_block +
              static_cast<float>(
                  speed_deg_per_microseconds_angle_azimuth * timing_offset_from_first_firing);

          if (angle_deg_azimuth_point >= 360.0f) {
            angle_deg_azimuth_point -= 360.0f;
          }

          angle_deg_azimuth_last = angle_deg_azimuth_point;
          float angle_rad_azimuth_point = utils::Utils::deg_to_rad(angle_deg_azimuth_point);
          float angle_deg_vertical = channel_to_angle_vertical_.at(ind_point);
          float angle_rad_vertical = utils::Utils::deg_to_rad(angle_deg_vertical);
          float dist_m = static_cast<float>(data_point.distance_divided_by_4mm * 4) / 1000.0f;

          float dist_xy = dist_m * std::cos(angle_rad_vertical);
          Point point;
          point.x = dist_xy * std::sin(angle_rad_azimuth_point);
          point.y = dist_xy * std::cos(angle_rad_azimuth_point);
          point.z = dist_m * std::sin(angle_rad_vertical);
          point.intensity = data_point.reflectivity;
          point.ring = ind_point;
          point.horizontal_angle = angle_deg_azimuth_point;
          point.stamp_unix_seconds =
              std::chrono::seconds(
                  tp_hours_since_epoch.time_since_epoch() + microseconds_since_toh.minutes() +
                  microseconds_since_toh.seconds())
                  .count();
          point.stamp_nanoseconds =
              std::chrono::nanoseconds(
                  microseconds_since_toh.subseconds() + timing_offset_from_first_block_time +
                  timing_offset_from_first_firing_time)
                  .count();

          if (dist_m != 0 && dist_m <= max_point_distance_from_lidar &&
              dist_m >= min_point_distance_from_lidar ) {
            cloud_.push_back(point);
          }
        }
      }
    }

    bool is_close_to_cut_area = std::abs(angle_deg_azimuth_last - angle_deg_cut_) < 1.0f;

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
}