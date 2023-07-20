// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/continuous_packet_parser.hpp"
#include <iostream>
#include <pcapplusplus/Packet.h>
#include "mapora/utils.hpp"

namespace mapora::points_provider::continuous_packet_parser
{
ContinuousPacketParser::ContinuousPacketParser(const size_t count_clouds_to_extract_max)
: count_clouds_to_extract_max_{count_clouds_to_extract_max},
  count_clouds_extracted_{0UL},
  factory_bytes_are_read_at_least_once_{false},
  has_received_valid_position_package_{false},
  has_processed_a_packet_{false},
  angle_deg_azimuth_last_packet_{0.0f},
  microseconds_last_packet_{0U},
  can_publish_again_{true},
  angle_deg_cut_{90.0f},
  skipped_cloud_first_{false}
{
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
    -11.742F, -1.99F, 3.4F, -5.29F, -0.78F, 4.61F, -4.08F, 1.31F, -6.5F, -1.11F,
    4.28F, -4.41F, 0.1F, 6.48F, -3.2F, 2.19F, -3.86F, 1.53F, -9.244F, -1.77F,
    2.74F, -5.95F, -0.56F, 4.83F, -2.98F, 2.41F, -6.28F, -0.89F, 3.62F, -5.07F,
    0.32F, 7.58F, -0.34F, 5.18F, -3.64F, 1.75F, -25.0F, -2.43F, 2.96F, -5.73F,
    0.54F, 9.7F, -2.76F, 2.63F, -7.65F, -1.55F, 3.84F, -4.85F, 3.18F, -5.51F,
    -0.12F, 5.73F, -4.3F, 1.09F, -16.042F, -2.21F, 4.06F, -4.63F, 0.76F, 15.0F,
    -3.42F, 1.97F, -6.85F, -1.33F, -5.62F, -0.23F, 5.43F, -3.53F, 0.98F, -19.582F,
    -2.32F, 3.07F, -4.74F, 0.65F, 11.75F, -2.65F, 1.86F, -7.15F, -1.44F, 3.95F,
    -2.1F, 3.29F, -5.4F, -0.01F, 4.5F, -4.19F, 1.2F, -13.565F, -1.22F, 4.17F,
    -4.52F, 0.87F, 6.08F, -3.31F, 2.08F, -6.65F, 1.42F, -10.346F, -1.88F, 3.51F,
    -6.06F, -0.67F, 4.72F, -3.97F, 2.3F, -6.39F, -1.0F, 4.39F, -5.18F, 0.21F,
    6.98F, -3.09F, 4.98F, -3.75F, 1.64F, -8.352F, -2.54F, 2.85F, -5.84F, -0.45F,
    8.43F, -2.87F, 2.52F, -6.17F, -1.66F, 3.73F, -4.96F, 0.43F};
  assert(channel_to_angle_vertical_.size() == 128);

  channel_to_ring_ = std::vector<size_t>(channel_to_angle_vertical_.size());
  std::iota(channel_to_ring_.begin(), channel_to_ring_.end(), 0);
  std::sort(
    channel_to_ring_.begin(), channel_to_ring_.end(), [this](size_t lhs, size_t rhs) {
      return channel_to_angle_vertical_.at(lhs) < channel_to_angle_vertical_.at(rhs);
    });

  clouds_rings_.resize(channel_to_angle_vertical_.size());
  //  for (size_t i = 0; i < channel_to_angle_vertical_.size(); ++i) {
  //    std::cout << channel_to_angle_vertical_.at(channel_to_ring_.at(i)) << std::endl;
  //  }

  channel_mod_8_to_azimuth_offsets_ =
    std::vector<float>{-6.354F, -4.548F, -2.732F, -0.911F, 0.911F, 2.732F, 4.548F, 6.354F};

  std::vector<size_t> vec_counting_numbers(12);
  std::iota(vec_counting_numbers.begin(), vec_counting_numbers.end(), 0);
  ind_block_to_first_channel_.resize(12);
  std::transform(
    vec_counting_numbers.begin(),
    vec_counting_numbers.end(),
    ind_block_to_first_channel_.begin(),
    [](size_t in) {return 32 * (in % 4);});
  std::cout << "vec_counting_numbers" << std::endl;
  std::copy(
    ind_block_to_first_channel_.begin(),
    ind_block_to_first_channel_.end(),
    std::ostream_iterator<int>(std::cout, " "));
}


void ContinuousPacketParser::process_packet_into_cloud(
  const pcpp::RawPacket & rawPacket,
  const std::function<void(const Points &)> & callback_cloud_surround_out)
{
  if (count_clouds_extracted_ > count_clouds_to_extract_max_ && count_clouds_to_extract_max_ != 0) {
    return;
  }
  switch (rawPacket.getFrameLength()) {
    case 554: {
        // Position Packet
        if (has_received_valid_position_package_) {break;}

        auto * position_packet = reinterpret_cast<const PositionPacket *>(rawPacket.getRawData());

        //        https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm


        std::string nmea_sentence(position_packet->nmea_sentence);
        auto segments_with_nullstuff = utils::Utils::string_to_vec_split_by(nmea_sentence, '\r');
        auto segments_with_crc =
          utils::Utils::string_to_vec_split_by(segments_with_nullstuff.front(), '*');
        auto segments = utils::Utils::string_to_vec_split_by(segments_with_crc.front(), ',');
        if (segments.size() != 14) {
          throw std::length_error(
                  "nmea sentence should have 14 elements, it has " +
                  std::to_string(segments.size()));
        }

        //          for (size_t i = 0; i < segments.size(); ++i) {
        //            std::cout << i << ": " << segments.at(i) << std::endl;
        //          }

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
        uint32_t seconds_epoch_precision_of_hour =
          std::chrono::seconds(tp_hours_since_epoch.time_since_epoch()).count();
        std::cout << "seconds_epoch_precision_of_hour: " << seconds_epoch_precision_of_hour <<
          std::endl;

        has_received_valid_position_package_ = true;
        break;
      }
    case 1248: {
        // Data Packet
        if (!has_received_valid_position_package_) {
          // Ignore until first valid Position Packet is received
          break;
        }
        const auto * data_packet_with_header =
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
        if (velodyne_model != VelodyneModel::VLS128) {
          throw std::runtime_error(
                  "velodyne_model was expected to be: " +
                  map_velodyne_model_to_string_.at(VelodyneModel::VLS128) +
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
                    "return_mode was expected to be: " + map_return_mode_to_string_.at(
                      return_mode_) +
                    " but it was: " + map_return_mode_to_string_.at(return_mode));
          }
        }

        // Iterate through 12 blocks
        double speed_deg_per_microseconds_angle_azimuth;
        float angle_deg_azimuth_last = 0.0f;
        for (size_t ind_block = 0; ind_block < data_packet_with_header->get_size_data_blocks();
          ind_block++)
        {
          const auto & data_block = data_packet_with_header->data_blocks[ind_block];
          float angle_deg_azimuth_of_block =
            static_cast<float>(data_block.azimuth_multiplied_by_100_deg) / 100.0f;
          //            std::cout <<
          //              "block: " << ind_block <<
          //              ", angle_deg_azimuth: " << angle_deg_azimuth_of_block << std::endl;

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

            uint32_t microseconds_toh_current_increased = data_packet_with_header->microseconds_toh;
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
            //              std::cout << "microseconds_delta: " <<
            //                microseconds_delta <<
            //                std::endl;
            //              std::cout << "angle_deg_angle_delta: " <<
            //                angle_deg_angle_delta <<
            //                std::endl;
            //              std::cout << "speed_deg_per_microseconds_angle_azimuth: " <<
            //                speed_deg_per_microseconds_angle_azimuth <<
            //                std::endl;
          }


          // Iterate through 32 points within a block
          for (size_t ind_point = 0; ind_point < data_block.get_size_data_points(); ind_point++) {
            const auto & data_point = data_block.data_points[ind_point];
            size_t channel = ind_block_to_first_channel_.at(ind_block) + ind_point;
            size_t firing_group = channel / 8;

            double timing_offset_from_first_firing_group =
              2.66666 * static_cast<int>(firing_group) - 7.0;
            float angle_deg_azimuth_point =
              angle_deg_azimuth_of_block +
              static_cast<float>(
              speed_deg_per_microseconds_angle_azimuth * timing_offset_from_first_firing_group);

            if (firing_group > 7) {
              angle_deg_azimuth_point +=
                static_cast<float>(speed_deg_per_microseconds_angle_azimuth * 5.33333);
            }
            angle_deg_azimuth_point -= channel_mod_8_to_azimuth_offsets_.at(channel % 8);

            if (angle_deg_azimuth_point >= 360.0f) {angle_deg_azimuth_point -= 360.0f;}

            angle_deg_azimuth_last = angle_deg_azimuth_point;
            float angle_rad_azimuth_point = utils::Utils::deg_to_rad(angle_deg_azimuth_point);

            //              std::cout <<
            //                "block: " << ind_block <<
            //                ", ind_point: " << ind_point <<
            //                ", channel: " << channel <<
            //                ", firing_group: " << firing_group <<
            //                ", angle_deg_azimuth: " << angle_deg_azimuth_of_block <<
            //                ", angle_deg_azimuth_point: " << angle_deg_azimuth_point <<
            //                ", timing_offset: " << timing_offset_from_first_firing_group <<
            //                std::endl;

            float angle_deg_vertical = channel_to_angle_vertical_.at(channel);
            float angle_rad_vertical = utils::Utils::deg_to_rad(angle_deg_vertical);
            float dist_m = static_cast<float>(data_point.distance_divided_by_4mm * 4) / 1000.0f;
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

            //  std::cout << "point stamp_unix_seconds: " << point.stamp_unix_seconds <<
            //    ", point stamp_nanoseconds: " << point.stamp_nanoseconds <<
            //    std::endl;
            if (std::sqrt(
                std::pow(
                  point.x,
                  2) + std::pow(point.y, 2) + std::pow(point.z, 2)) > 100)
            {
              continue;
            }

            clouds_rings_.at(channel_to_ring_.at(channel)).emplace_back(point);
            //            cloud_.push_back(point);
          }
        }

        bool is_close_to_cut_area = std::abs(angle_deg_azimuth_last - angle_deg_cut_) < 1.0f;

        if (!is_close_to_cut_area) {can_publish_again_ = true;}

        if (can_publish_again_ && is_close_to_cut_area) {

          cloud_.clear();
          for (const auto & cloud_ring : clouds_rings_) {
            cloud_.insert(cloud_.begin(), cloud_ring.cbegin(), cloud_ring.cend());
          }

          if(!skipped_cloud_first_){
            // skip first cloud
            skipped_cloud_first_ = true;
          }else{
            callback_cloud_surround_out(cloud_);
            count_clouds_extracted_++;
          }

          std::for_each(
            clouds_rings_.begin(), clouds_rings_.end(), [](Points & points) {points.clear();});
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
}  // namespace mapora::points_provider::continuous_packet_parser
