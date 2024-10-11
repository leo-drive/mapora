#ifndef BUILD_POINT_TYPES_HPP
#define BUILD_POINT_TYPES_HPP

#include <boost/math/special_functions/relative_difference.hpp>

#include <limits>
#include <numeric>

namespace mapora::point_types
{
struct PointXYZI
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  friend bool operator==(const PointXYZI & p1, const PointXYZI & p2)
  {
    using boost::math::epsilon_difference;
    return epsilon_difference(p1.x, p2.x) == 0.0F && epsilon_difference(p1.y, p2.y) == 0.0F &&
           epsilon_difference(p1.z, p2.z) == 0.0F &&
           epsilon_difference(p1.intensity, p2.intensity) == 0.0F;
  }
} __attribute__((packed));

struct PointXYZIT
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  uint32_t intensity{0U};
  uint32_t stamp_unix_seconds{0U};
  uint32_t stamp_nanoseconds{0U};
  friend bool operator==(const PointXYZIT & p1, const PointXYZIT & p2)
  {
    using boost::math::epsilon_difference;
    return epsilon_difference(p1.x, p2.x) == 0.0F && epsilon_difference(p1.y, p2.y) == 0.0F &&
           epsilon_difference(p1.z, p2.z) == 0.0F && p1.intensity == p2.intensity &&
           p1.stamp_unix_seconds == p2.stamp_unix_seconds &&
           p1.stamp_nanoseconds == p2.stamp_nanoseconds;
  }
} __attribute__((packed));

struct PointXYZIR
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  uint32_t intensity{0U};
  uint32_t ring{0U};
  friend bool operator==(const PointXYZIR & p1, const PointXYZIR & p2)
  {
    using boost::math::epsilon_difference;
    return epsilon_difference(p1.x, p2.x) == 0.0F && epsilon_difference(p1.y, p2.y) == 0.0F &&
           epsilon_difference(p1.z, p2.z) == 0.0F && p1.intensity == p2.intensity &&
           p1.ring == p2.ring;
  }
} __attribute__((packed));

struct PointXYZITRH
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  uint32_t intensity{0U};
  uint32_t stamp_unix_seconds{0U};
  uint32_t stamp_nanoseconds{0U};
  uint32_t ring{0U};
  float horizontal_angle{0.0F};
  friend bool operator==(const PointXYZITRH & p1, const PointXYZITRH & p2)
  {
    using boost::math::epsilon_difference;
    return epsilon_difference(p1.x, p2.x) == 0.0F && epsilon_difference(p1.y, p2.y) == 0.0F &&
           epsilon_difference(p1.z, p2.z) == 0.0F && p1.intensity == p2.intensity &&
           p1.stamp_unix_seconds == p2.stamp_unix_seconds &&
           p1.stamp_nanoseconds == p2.stamp_nanoseconds && p1.ring == p2.ring;
  }
} __attribute__((packed));
}  // namespace loam_mapper::point_types

#endif  // BUILD_POINT_TYPES_HPP
