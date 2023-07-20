// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

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
