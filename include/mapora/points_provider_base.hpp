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

#ifndef MAPORA__POINTS_PROVIDER_BASE_HPP_
#define MAPORA__POINTS_PROVIDER_BASE_HPP_

#include <cstdint>
#include <vector>
#include <string>
#include "point_xyzit.hpp"

namespace mapora::points_provider
{
class PointsProviderBase
{
public:
  using Point = point_types::PointXYZIT;
  using Points = std::vector<Point>;
  virtual void process() = 0;
//  virtual bool get_next_cloud(std::vector<Points> & cloud_out) = 0;
  virtual std::string info() = 0;
};
}  // namespace mapora::points_provider

#endif  // MAPORA__POINTS_PROVIDER_BASE_HPP_
