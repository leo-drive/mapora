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

#ifndef MAPORA__UTILS_HPP_
#define MAPORA__UTILS_HPP_

#include <string>
#include <cstdint>
#include <vector>
#include <cmath>
#include <type_traits>

namespace mapora::utils
{

class Utils
{
public:
  static std::string byte_hex_to_string(uint8_t byte_hex);
  static std::string bytes_hexes_to_string(const std::vector<uint8_t> & bytes_hexes);
  static std::vector<std::string> string_to_vec_split_by(
    const std::string & input,
    char splitter);

  template<typename T>
  static T deg_to_rad(T deg)
  {
    constexpr double multiplier = M_PI / 180.0;
    return static_cast<T>(deg * multiplier);
  }
};

}  // namespace mapora::utils

#endif  // MAPORA__UTILS_HPP_
