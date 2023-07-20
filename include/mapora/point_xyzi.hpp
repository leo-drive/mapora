// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__POINT_XYZI_HPP_
#define MAPORA__POINT_XYZI_HPP_

#include <boost/math/special_functions/relative_difference.hpp>
#include <numeric>
#include <limits>

namespace mapora::point_types
{
struct PointXYZI
{
  float x {0.0F};
  float y {0.0F};
  float z {0.0F};
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
