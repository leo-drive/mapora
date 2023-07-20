// Copyright (c) 2021 Leo Drive Teknoloji A.Ş.
// All rights reserved.

#ifndef MAPORA__TIME_KEEPER_SEQUENTIAL_HPP_
#define MAPORA__TIME_KEEPER_SEQUENTIAL_HPP_

#include <chrono>
#include <utility>
#include <vector>
#include <string>
#include <iostream>

namespace time_keeper_sequential
{
class TimeKeeperSequential
{
public:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = Clock::time_point;

  explicit TimeKeeperSequential(std::string name)
  : name_(std::move(name)) {}

  static double millis_passed_between(const TimePoint & t1, const TimePoint & t2)
  {
    return std::chrono::duration<double>(t2 - t1).count() * 1000.0;
  }

  static double millis_passed_since(const TimePoint & t)
  {
    return std::chrono::duration<double>(Clock::now() - t).count() * 1000.0;
  }

  void add_time_point(const std::string & name)
  {
    vec_timepoints_.emplace_back(TimeKeeperSequential::Clock::now());
    vec_timepoint_names_.emplace_back(name);
  }

  void print_times()
  {
    for (size_t i = 1; i < vec_timepoints_.size(); ++i) {
      std::cout << name_ << " - " << vec_timepoint_names_[i] << " took " <<
        millis_passed_between(vec_timepoints_[i - 1], vec_timepoints_[i]) << " ms." <<
        std::endl;
    }

    std::cout << name_ << " took TOTAL " <<
      millis_passed_between(vec_timepoints_.front(), vec_timepoints_.back()) << " ms." <<
      std::endl;
  }

private:
  std::string name_;
  std::vector<TimePoint> vec_timepoints_;
  std::vector<std::string> vec_timepoint_names_;
};
}  // namespace time_keeper_sequential

#endif  // MAPORA__TIME_KEEPER_SEQUENTIAL_HPP_
