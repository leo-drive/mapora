// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__POINT_XYZIT_HPP_
#define MAPORA__POINT_XYZIT_HPP_

#include <boost/math/special_functions/relative_difference.hpp>
#include <numeric>
#include <cstdint>
#include <limits>

namespace mapora::point_types
{
struct PointXYZIT
{
  float x {0.0F};
  float y {0.0F};
  float z {0.0F};
  uint32_t intensity {0U};
  uint32_t stamp_unix_seconds{0U};
  uint32_t stamp_nanoseconds{0U};
  friend bool operator==(
    const PointXYZIT & p1,
    const PointXYZIT & p2)
  {
    using boost::math::epsilon_difference;
    return epsilon_difference(p1.x, p2.x) == 0.0F &&
           epsilon_difference(p1.y, p2.y) == 0.0F &&
           epsilon_difference(p1.z, p2.z) == 0.0F &&
           p1.intensity == p2.intensity &&
           p1.stamp_unix_seconds == p2.stamp_unix_seconds &&
           p1.stamp_nanoseconds == p2.stamp_nanoseconds;
  }
} __attribute__((packed));
}  // namespace mapora::point_types


#endif  // MAPORA__POINT_XYZIT_HPP_
