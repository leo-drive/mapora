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
#include "mapora/utils.hpp"

#include <iomanip>
#include <string>
#include <sstream>
#include <vector>

namespace mapora::utils
{

std::string Utils::byte_hex_to_string(uint8_t byte_hex)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  ss << "data_packet_with_header: " << std::setw(2) << (int)byte_hex;
  return ss.str();
}

std::string Utils::bytes_hexes_to_string(const std::vector<uint8_t> & bytes_hexes)
{
  std::string output;
  for (const auto & byte_hex : bytes_hexes) {
    output += byte_hex_to_string(byte_hex) + " ";
  }
  if (output.empty()) {
    throw std::length_error("output.empty()");
  }
  output.erase(output.end() - 1, output.end());
  return output;
}
std::vector<std::string> Utils::string_to_vec_split_by(
  const std::string & input, char splitter)
{
  std::stringstream ss_input(input);
  std::string segment;
  std::vector<std::string> seglist;
  while (std::getline(ss_input, segment, splitter)) {
    seglist.push_back(segment);
  }
  return seglist;
}



}  // namespace mapora::utils
