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

#include "mapora/transform_provider_applanix.hpp"
#include "mapora/csv.hpp"
#include <string>
#include <array>
#include <fstream>
#include <exception>
#include <algorithm>
#include <iostream>
#include <Eigen/Geometry>
#include <GeographicLib/TransverseMercatorExact.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include "mapora/date.h"
#include "mapora/utils.hpp"

namespace mapora::transform_provider_applanix {
TransformProviderApplanix::TransformProviderApplanix(
  const std::string &path_file_ascii_output)
  : path_file_ascii_output_(path_file_ascii_output) {
  if (!fs::exists(path_file_ascii_output_)) {
    throw std::runtime_error(
      "path_file_ascii_output doesn't exist: " + path_file_ascii_output_.string());
  }
  if (!fs::is_regular_file(path_file_ascii_output_)) {
    throw std::runtime_error(
      "path_file_ascii_output is not a file path: " + path_file_ascii_output_.string());
  }
}

void TransformProviderApplanix::process() {
  io::LineReader lines(path_file_ascii_output_.string());
  double line_number_for_header = 1;
  while (char *line = lines.next_line())
    // make the header line a readable header line..
  {
    if (line_number_for_header == 16) {  // 16 is the mission date line in the applanix export file
      mission_date += line;
      mission_date.erase(0, 20);  // erases 20 characters from 0.
      std::cout << "mission_date: " << mission_date << std::endl;
      break;
    }

    if (line_number_for_header == 29) {  // 29 is the first data line. UTC Time is wanted
      time_string += line;
      time_string.erase(11, 188);  // only the time left in the string
      break;
    }
    line_number_for_header++;
  }

  std::string path_applanix_modified = "temp_applanix_modified.csv";
  {
    std::ifstream filein(path_file_ascii_output_.string()); //File to read from
    std::ofstream fileout(path_applanix_modified); //Temporary file
    if (!filein || !fileout) {
      // throw exception
      throw (std::runtime_error("Error opening file"));
    }

    // Iterate the file and write to the temporary file
    std::string line;
    bool header_is_found = false;
    size_t line_number_header = 0;
    size_t line_number_current = 0;
    while (std::getline(filein, line)) {
      line_number_current++;

      // fix the header line
      if (line.find("TIME,") != std::string::npos) {
        line_number_header = line_number_current;
        header_is_found = true;

        // separate line into tokens by comma
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
          // remove leading and trailing spaces
          token.erase(0, token.find_first_not_of(' '));
          token.erase(token.find_last_not_of(' ') + 1);
          // replace spaces with underscores
          std::replace(token.begin(), token.end(), ' ', '_');
          tokens.push_back(token);
        }
        // concatenate tokens with spaces
        std::string new_line;
        for (const auto &token : tokens) {
          new_line += token + ",";
        }
        // remove last comma
        new_line.pop_back();
        line = new_line;
      }

      // fix the data lines to be csv like
      if (line_number_current > line_number_header + 3 && header_is_found) {

        // replace multiple spaces with single space
        line.erase(std::unique(line.begin(), line.end(),
                               [](char a, char b) { return isspace(a) && isspace(b); }),
                   line.end());
        // replace spaces with commas
        std::replace(line.begin(), line.end(), '\t', ',');
        line.pop_back();
      }
      fileout << line << std::endl;
    }
  }


  io::CSVReader<26, io::trim_chars<','>> csv_global_pose(path_applanix_modified);
  for (int i = 0; i < 24; ++i)
    csv_global_pose.next_line();

  // take only what you need
  csv_global_pose.read_header(
//                line_char,
    io::ignore_no_column,
    "TIME",
    "DISTANCE",
    "EASTING",
    "NORTHING",
    "ORTHOMETRIC_HEIGHT",
    "LATITUDE",
    "LONGITUDE",
    "ELLIPSOID_HEIGHT",
    "ROLL",
    "PITCH",
    "HEADING",
    "EAST_VELOCITY",
    "NORTH_VELOCITY",
    "UP_VELOCITY",
    "X_ANGULAR_RATE",
    "Y_ANGULAR_RATE",
    "Z_ANGULAR_RATE",
    "X_ACCELERATION",
    "Y_ACCELERATION",
    "Z_ACCELERATION",
    "EAST_SD",
    "NORTH_SD",
    "HEIGHT_SD",
    "ROLL_SD",
    "PITCH_SD",
    "HEADING_SD"
  );
  // Skip "units" line
  csv_global_pose.next_line();
  csv_global_pose.next_line();
  csv_global_pose.next_line();


  struct ApplanixInput {
    double utc_time;            // in seconds
    double distance;            // in meters
    double easting;             // in meters
    double northing;            // in meters
    double orthometric_height;  // in meters
    double latitude;            // in degrees
    double longitude;           // in degrees
    double ellipsoid_height;    // in meters
    double roll;                // in degrees
    double pitch;               // in degrees
    double heading;             // in degrees
    double east_vel;            // in Meter/Sec
    double north_vel;           // in Meter/Sec
    double up_vel;              // in Meter/Sec
    double x_angular_rate;            // in meters
    double y_angular_rate;            // in meters
    double z_angular_rate;            // in meters
    double x_acceleration;            // in meters
    double y_acceleration;            // in meters
    double z_acceleration;            // in meters

    double east_std;            // in meters
    double north_std;           // in meters
    double height_std;          // in meters
    double roll_std;            // in degrees
    double pitch_std;           // in degrees
    double heading_std;         // in degrees
  } in;

  try {
    data_line_number = csv_global_pose.get_file_line() + 1;
    while (csv_global_pose.read_row(
      in.utc_time,
      in.distance,
      in.easting,
      in.northing,
      in.orthometric_height,
      in.latitude,
      in.longitude,
      in.ellipsoid_height,
      in.roll,
      in.pitch,
      in.heading,
      in.east_vel,
      in.north_vel,
      in.up_vel,
      in.x_angular_rate,
      in.y_angular_rate,
      in.z_angular_rate,
      in.x_acceleration,
      in.y_acceleration,
      in.z_acceleration,
      in.east_std,
      in.north_std,
      in.height_std,
      in.roll_std,
      in.pitch_std,
      in.heading_std)) {

      Pose pose;
      Imu imu;
      pose.pose_with_covariance.pose.position.set__x(in.easting);
      pose.pose_with_covariance.pose.position.set__y(in.northing);
      pose.pose_with_covariance.pose.position.set__z(in.ellipsoid_height);
      Eigen::AngleAxisd angle_axis_x(utils::Utils::deg_to_rad(in.roll), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd angle_axis_y(utils::Utils::deg_to_rad(in.pitch), Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd angle_axis_z(utils::Utils::deg_to_rad(-in.heading), Eigen::Vector3d::UnitZ());
      pose.velocity.x = in.east_vel;
      pose.velocity.y = in.north_vel;
      pose.velocity.z = in.up_vel;

      Eigen::Quaterniond q = (angle_axis_z * angle_axis_y * angle_axis_x);
      pose.pose_with_covariance.pose.orientation.set__x(q.x());
      pose.pose_with_covariance.pose.orientation.set__y(q.y());
      pose.pose_with_covariance.pose.orientation.set__z(q.z());
      pose.pose_with_covariance.pose.orientation.set__w(q.w());

      imu.imu.linear_acceleration.set__x(in.y_acceleration);
      imu.imu.linear_acceleration.set__y(in.x_acceleration);
      imu.imu.linear_acceleration.set__z(-in.z_acceleration);
      std::array<double, 3> linear_acc_variances{
          std::pow(in.east_std, 2),  std::pow(-in.north_std, 2), std::pow(-in.height_std, 2),
      };
      for (size_t i = 0; i < 3; ++i) {
        imu.imu.linear_acceleration_covariance.at(i*4) = linear_acc_variances.at(i);
      }

      imu.imu.angular_velocity.set__x(in.y_angular_rate);
      imu.imu.angular_velocity.set__y(in.x_angular_rate);
      imu.imu.angular_velocity.set__z(-in.z_angular_rate);
      std::array<double, 3> angular_rate_variances{
          std::pow(in.pitch_std, 2),  std::pow(in.roll_std, 2), std::pow(-in.heading_std, 2),
      };
      for (size_t i = 0; i < 3; ++i) {
        imu.imu.linear_acceleration_covariance.at(i*4) = linear_acc_variances.at(i);
      }


      auto segments = utils::Utils::string_to_vec_split_by(mission_date, '/');

      int days_raw = std::stoi(segments.at(0));
      int months_raw = std::stoi(segments.at(1));
      int years_raw = std::stoi(segments.at(2));

      date::year_month_day date_current_ = date::year{years_raw} / months_raw / days_raw;
      date::hh_mm_ss time_since_midnight = date::make_time(
        std::chrono::milliseconds(static_cast<uint64_t>(in.utc_time * 1000)));

      auto tp = date::sys_days(date_current_) +
                std::chrono::hours{time_since_midnight.hours()} +
                std::chrono::minutes{time_since_midnight.minutes()} +
                std::chrono::seconds{time_since_midnight.seconds()};

      pose.stamp_unix_seconds = std::chrono::seconds(tp.time_since_epoch()).count();
      pose.stamp_nanoseconds = std::chrono::nanoseconds(time_since_midnight.subseconds()).count();

      std::array<double, 6> variances{
        std::pow(in.north_std, 2),
        std::pow(in.east_std, 2),
        std::pow(in.height_std, 2),
        std::pow(in.roll_std, 2),
        std::pow(in.pitch_std, 2),
        std::pow(in.heading_std, 2),
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
      pose.meridian_convergence = compute_meridian_convergence(in.latitude, in.longitude);
      imu_rotations_.push_back(imu);

      poses_.push_back(pose);
    }
  } catch (const std::exception &ex) {
    std::cerr << "Probably empty lines at the end of csv, no problems: " << ex.what() << std::endl;
  }
}

TransformProviderApplanix::Pose TransformProviderApplanix::get_pose_at(
  uint32_t stamp_unix_seconds,
  uint32_t stamp_nanoseconds) {
  Pose pose_search;
  pose_search.stamp_unix_seconds = stamp_unix_seconds;
  pose_search.stamp_nanoseconds = stamp_nanoseconds;
  auto iter_result = std::lower_bound(
    poses_.begin(), poses_.end(), pose_search,
    [](const Pose &p1, const Pose &p2) {
      if (p1.stamp_unix_seconds == p2.stamp_unix_seconds) {
        return p1.stamp_nanoseconds < p2.stamp_nanoseconds;
      }
      return p1.stamp_unix_seconds < p2.stamp_unix_seconds;
    });
  size_t index = std::distance(poses_.begin(), iter_result);
  return poses_.at(index);
}

TransformProviderApplanix::Imu TransformProviderApplanix::get_imu_at(
    uint32_t stamp_unix_seconds, uint32_t stamp_nanoseconds)
{
  Imu imu_search;

  imu_search.stamp_unix_seconds = stamp_unix_seconds;
  imu_search.stamp_nanoseconds = stamp_nanoseconds;
  auto iter_result = std::lower_bound(
      imu_rotations_.begin(), imu_rotations_.end(), imu_search, [](const Imu & p1, const Imu & p2) {
        if (p1.stamp_unix_seconds == p2.stamp_unix_seconds) {
          return p1.stamp_nanoseconds < p2.stamp_nanoseconds;
        }
        return p1.stamp_unix_seconds < p2.stamp_unix_seconds;
      });

  size_t index;

  if (
      iter_result == imu_rotations_.end() ||
      iter_result->stamp_unix_seconds > stamp_unix_seconds + 1 ||
      iter_result->stamp_nanoseconds > stamp_nanoseconds + 50000000) {
    //    std::cerr << "Imu not found for timestamp: "
    //              << stamp_unix_seconds << "." << stamp_nanoseconds << std::endl;
    // Handle the error as needed, e.g., throw an exception, return a default pose, etc.
    //    throw std::runtime_error("Imu not found");
    index = last_index_imu;
  } else {
    index = std::distance(imu_rotations_.begin(), iter_result);
    last_index_imu = index;
  }


  //  std::cout << "ind: " << index << std::endl;
  return imu_rotations_.at(index);
}
std::vector<double> TransformProviderApplanix::parse_mgrs_coordinates(const std::string & mgrs_string) {
  std::string mgrs_grid = mgrs_string.substr(0, 5);
  std::string mgrs_x_str = mgrs_string.substr(5, 8);
  std::string mgrs_y_str = mgrs_string.substr(13, 8);

  double mgrs_x = std::stod(mgrs_x_str);
  mgrs_x /= 1000;

  double mgrs_y = std::stod(mgrs_y_str);
  mgrs_y /= 1000;

  return std::vector{mgrs_x, mgrs_y};
}

std::string TransformProviderApplanix::parse_mgrs_zone(const std::string & mgrs_string) {
  return mgrs_string.substr(0, 5);
}

float TransformProviderApplanix::compute_meridian_convergence(double lat, double lon) {

  double x, y;
  int zone;
  bool northp;

  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
  double lambda0 = (zone - 1) * 6 - 180 + 3;

  GeographicLib::TransverseMercatorExact tm(
      GeographicLib::Constants::WGS84_a(),
      GeographicLib::Constants::WGS84_f(),
      lambda0);

  double gamma, k;
  tm.Forward(lambda0, lat, lon, x, y, gamma, k);

  return gamma;
}

}  // mapora::transform_provider