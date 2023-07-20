// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/transform_provider_sbg.hpp"
#include "mapora/csv.hpp"
#include <string>
#include <array>
#include <sstream>
#include <exception>
#include <algorithm>
#include <iostream>
#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include "mapora/date.h"
#include "mapora/utils.hpp"

namespace mapora::transform_provider
{
TransformProviderSbg::TransformProviderSbg(const std::string & path_file_ascii_output)
: path_file_ascii_output_(path_file_ascii_output)
{
  if (!fs::exists(path_file_ascii_output_)) {
    throw std::runtime_error(
            "path_file_ascii_output doesn't exist: " + path_file_ascii_output_.string());
  }
  if (!fs::is_regular_file(path_file_ascii_output_)) {
    throw std::runtime_error(
            "path_file_ascii_output is not a file path: " + path_file_ascii_output_.string());
  }
}

void TransformProviderSbg::process()
{
  io::CSVReader<18> csv_global_pose(path_file_ascii_output_.string());
  for (int i = 0; i < 32; ++i) {
    csv_global_pose.next_line();
  }


  csv_global_pose.read_header(
    io::ignore_extra_column,
    "UTC Date",
    "UTC Time",
    "q0",
    "q1",
    "q2",
    "q3",
    "Roll",
    "Pitch",
    "Yaw",
    "Roll Std.",
    "Pitch Std.",
    "Yaw Std.",
    "Latitude",
    "Longitude",
    "Altitude MSL",
    "Latitude Std.",
    "Longitude Std.",
    "Altitude Std.");

  // Skip "units" line
  csv_global_pose.next_line();

  struct SbgInput
  {
    std::string utc_date;
    double utc_time;
    double quat_w;
    double quat_x;
    double quat_y;
    double quat_z;
    double roll;
    double pitch;
    double yaw;
    double roll_std;
    double pitch_std;
    double yaw_std;
    double latitude;
    double longitude;
    double altitude_msl;
    double latitude_std;
    double longitude_std;
    double altitude_msl_std;
  } in;


  GeographicLib::LocalCartesian local_cartesian;

  bool init = false;
  try {
    while (csv_global_pose.read_row(
        in.utc_date,
        in.utc_time,
        in.quat_w,
        in.quat_x,
        in.quat_y,
        in.quat_z,
        in.roll,
        in.pitch,
        in.yaw,
        in.roll_std,
        in.pitch_std,
        in.yaw_std,
        in.latitude,
        in.longitude,
        in.altitude_msl,
        in.latitude_std,
        in.longitude_std,
        in.altitude_msl_std))
    {
      Pose pose;
      if (!init) {
        local_cartesian = GeographicLib::LocalCartesian(in.latitude, in.longitude, in.altitude_msl);
        init = true;
        pose.pose_with_covariance.pose.position.set__x(0.0f);
        pose.pose_with_covariance.pose.position.set__y(0.0f);
        pose.pose_with_covariance.pose.position.set__z(0.0f);
      } else {
        double cart_x;
        double cart_y;
        double cart_z;
        local_cartesian.Forward(in.latitude, in.longitude, in.altitude_msl, cart_x, cart_y, cart_z);
        //      std::cout <<
        //        "cart_x: " << cart_x <<
        //        ", cart_y: " << cart_y <<
        //        ", cart_z: " << cart_z <<
        //        std::endl;
        pose.pose_with_covariance.pose.position.set__x(cart_x);
        pose.pose_with_covariance.pose.position.set__y(cart_y);
        pose.pose_with_covariance.pose.position.set__z(cart_z);
      }
//      Eigen::AngleAxisd angle_axis_x(utils::Utils::deg_to_rad(in.roll), Eigen::Vector3d::UnitX());
//      Eigen::AngleAxisd angle_axis_y(utils::Utils::deg_to_rad(in.pitch), Eigen::Vector3d::UnitY());
//      Eigen::AngleAxisd angle_axis_z(utils::Utils::deg_to_rad(in.yaw), Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = Eigen::Quaterniond(in.quat_w, in.quat_x, in.quat_y, in.quat_z).inverse();
//      q = (angle_axis_z * angle_axis_y * angle_axis_x).inverse();

      pose.pose_with_covariance.pose.orientation.set__x(q.x());
      pose.pose_with_covariance.pose.orientation.set__y(q.y());
      pose.pose_with_covariance.pose.orientation.set__z(q.z());
      pose.pose_with_covariance.pose.orientation.set__w(q.w());

//      pose.pose_with_covariance.pose.orientation.set__w(in.quat_w);
//      pose.pose_with_covariance.pose.orientation.set__x(in.quat_x);
//      pose.pose_with_covariance.pose.orientation.set__y(in.quat_y);
//      pose.pose_with_covariance.pose.orientation.set__z(in.quat_z);


      auto segments = utils::Utils::string_to_vec_split_by(in.utc_date, '-');

      int years_raw = std::stoi(segments.at(0));
      int months_raw = std::stoi(segments.at(1));
      int days_raw = std::stoi(segments.at(2));

      date::year_month_day date_current_ = date::year{years_raw} / months_raw / days_raw;
      date::hh_mm_ss time_since_midnight = date::make_time(
        std::chrono::milliseconds(static_cast<uint64_t>(in.utc_time * 1000)) - date::days(1));

      auto tp = date::sys_days(date_current_) + std::chrono::hours{time_since_midnight.hours()} +
      std::chrono::minutes{time_since_midnight.minutes()} +
      std::chrono::seconds{time_since_midnight.seconds()};

      pose.stamp_unix_seconds = std::chrono::seconds(tp.time_since_epoch()).count();
      pose.stamp_nanoseconds = std::chrono::nanoseconds(time_since_midnight.subseconds()).count();

      //      std::cout << "sec: " << pose.stamp_unix_seconds <<
      //        ", nanosec: " << pose.stamp_nanoseconds <<
      //        std::endl;
      //    std::stringstream sstream;
      //    sstream <<
      //      "in_utc_date: " << in.utc_date <<
      //      ", in_utc_time: " << in.utc_time <<
      //      ", in_roll: " << in.roll <<
      //      ", in_pitch: " << in.pitch <<
      //      ", in_yaw: " << in.yaw <<
      //      ", in_roll_std: " << in.roll_std <<
      //      ", in_pitch_std: " << in.pitch_std <<
      //      ", in_yaw_std: " << in.yaw_std <<
      //      ", in_latitude: " << in.latitude <<
      //      ", in_longitude: " << in.longitude <<
      //      ", in_altitude_msl: " << in.altitude_msl <<
      //      ", in_latitude_std: " << in.latitude_std <<
      //      ", in_longitude_std: " << in.longitude_std <<
      //      ", in_altitude_msl_std: " << in.altitude_msl_std;
      //    std::cout << sstream.str() << std::endl;

      std::array<double, 6> variances{
        std::pow(in.latitude_std, 2),
        std::pow(in.longitude_std, 2),
        std::pow(in.altitude_msl_std, 2),
        std::pow(in.roll_std, 2),
        std::pow(in.pitch_std, 2),
        std::pow(in.yaw_std, 2),
      };

      //  0  1  2  3  4  5
      //  6  7  8  9  10 11
      //  12 13 14 15 16 17
      //  18 19 20 21 22 23
      //  24 25 26 27 28 29
      //  30 31 32 33 34 35
      //  fill diagonal with variances
      for (size_t i = 0; i < 6; ++i) {
        pose.pose_with_covariance.covariance.at(i * 7) = variances.at(i);
      }

      poses_.push_back(pose);
    }
  } catch (const std::exception & ex) {
    std::cerr << "Probably empty lines at the end of csv, no problems: " << ex.what() << std::endl;
  }

  //  auto ss = get_pose_at(1623676181, 829000000);
  //  exit(0);
}

TransformProviderSbg::Pose TransformProviderSbg::get_pose_at(
  uint32_t stamp_unix_seconds, uint32_t stamp_nanoseconds)
{
  Pose pose_search;
  pose_search.stamp_unix_seconds = stamp_unix_seconds;
  pose_search.stamp_nanoseconds = stamp_nanoseconds;
  auto iter_result = std::lower_bound(
    poses_.begin(), poses_.end(), pose_search, [](const Pose & p1, const Pose & p2) {
      if (p1.stamp_unix_seconds == p2.stamp_unix_seconds) {
        return p1.stamp_nanoseconds < p2.stamp_nanoseconds;
      }
      return p1.stamp_unix_seconds < p2.stamp_unix_seconds;
    });
  if (iter_result == poses_.end()) {
    // last item
    return poses_.at(poses_.size() - 1);
  }

  size_t index = std::distance(poses_.begin(), iter_result);
  //  std::cout << "ind: " << index << std::endl;
  return poses_.at(index);
}

}  // namespace mapora::transform_provider
