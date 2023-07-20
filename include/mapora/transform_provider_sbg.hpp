// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__TRANSFORM_PROVIDER_SBG_HPP_
#define MAPORA__TRANSFORM_PROVIDER_SBG_HPP_

#include <boost/filesystem.hpp>
#include <string>
#include <memory>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

namespace mapora::transform_provider
{
namespace fs = boost::filesystem;
class TransformProviderSbg
{
public:
  using SharedPtr = std::shared_ptr<TransformProviderSbg>;
  using ConstSharedPtr = const SharedPtr;

  explicit TransformProviderSbg(const std::string & path_file_ascii_output);

  void process();

  struct Pose
  {
    uint32_t stamp_unix_seconds{0U};
    uint32_t stamp_nanoseconds{0U};
    geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  };

  std::vector<Pose> poses_;

  Pose get_pose_at(
    uint32_t stamp_unix_seconds,
    uint32_t stamp_nanoseconds);

private:
  fs::path path_file_ascii_output_;
};
}  // mapora::transform_provider

#endif  // MAPORA__TRANSFORM_PROVIDER_SBG_HPP_
