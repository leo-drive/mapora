// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__UTILS_HPP_
#define MAPORA__UTILS_HPP_

#include <string>
#include <cstdint>
#include <vector>
#include <cmath>
#include <type_traits>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Geometry>

namespace mapora::utils
{
class Utils
{
public:
  static std::string byte_hex_to_string(uint8_t byte_hex);
  static std::string bytes_hexes_to_string(const std::vector<uint8_t> & bytes_hexes);
  static std::vector<std::string> string_to_vec_split_by(const std::string & input, char splitter);

  template<typename T>
  static T deg_to_rad(T deg)
  {
    constexpr double multiplier = M_PI / 180.0;
    return static_cast<T>(deg * multiplier);
  }

  template<typename T>
  static T rad_to_deg(T rad)
  {
    constexpr double multiplier =  180.0 / M_PI;
    return static_cast<T>(rad * multiplier);
  }

  static Eigen::Matrix4d pose_to_mat_eigen(const geometry_msgs::msg::Pose & pose);
};

}  // namespace mapora::utils

#endif  // MAPORA__UTILS_HPP_
