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
#ifndef MAPORA__POINT_XYZI_HPP_
#define MAPORA__POINT_XYZI_HPP_

#include <boost/math/special_functions/relative_difference.hpp>
#include <numeric>
#include <limits>

namespace mapora::point_types
{
struct PointXYZI
{
  double x;
  double y;
  double z;
  float intensity {0.0F};
  friend bool operator==(
    const PointXYZI & p1,
    const PointXYZI & p2)
  {
    using boost::math::epsilon_difference;
    return epsilon_difference(p1.x, p2.x) == 0.0F &&
           epsilon_difference(p1.y, p2.y) == 0.0F &&
           epsilon_difference(p1.z, p2.z) == 0.0F &&
           epsilon_difference(p1.intensity, p2.intensity) == 0.0F;
  }
} __attribute__((packed));
}  // namespace mapora::point_types


#endif  // MAPORA__POINT_XYZI_HPP_
