// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#include "mapora/utils.hpp"

#include <iomanip>
#include <string>
#include <sstream>
#include <cstdint>
#include <vector>
#include <exception>

namespace mapora::utils
{
std::string Utils::byte_hex_to_string(uint8_t byte_hex)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  ss << "data_packet_with_header: " << std::setw(2) << static_cast<int>(byte_hex);
  return ss.str();
}

std::string Utils::bytes_hexes_to_string(const std::vector<uint8_t> & bytes_hexes)
{
  std::string output;
  for (const auto & byte_hex : bytes_hexes) {
    output += byte_hex_to_string(byte_hex) + " ";
  }
  if (output.empty()) {throw std::length_error("output.empty()");}
  output.erase(output.end() - 1, output.end());
  return output;
}
std::vector<std::string> Utils::string_to_vec_split_by(const std::string & input, char splitter)
{
  std::stringstream ss_input(input);
  std::string segment;
  std::vector<std::string> seglist;
  while (std::getline(ss_input, segment, splitter)) {seglist.push_back(segment);}
  return seglist;
}

Eigen::Matrix4d Utils::pose_to_mat_eigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  const auto quat_in = pose.orientation;
  Eigen::Quaterniond quat(quat_in.w, quat_in.x, quat_in.y, quat_in.z);
  matrix.topLeftCorner<3, 3>() = quat.toRotationMatrix();
  const auto &translation_in = pose.position;
  matrix.topRightCorner<3, 1>() =
    Eigen::Vector3d(translation_in.x, translation_in.y, translation_in.z);
  return matrix;
}


}  // namespace mapora::utils
