#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include <rclcpp/rclcpp.hpp>

namespace mid360_deskew
{

struct DelayStats
{
  std::size_t sample_count{0};
  double mean{0.0};
  double stddev{0.0};
  double min{0.0};
  double max{0.0};
};

class DelayStatistics
{
public:
  void addSample(const rclcpp::Time & received_time, double delay_seconds, double window_seconds)
  {
    if (!samples_.empty() && received_time < samples_.front().received_time) {
      samples_.clear();
    }

    samples_.push_back({received_time, delay_seconds});
    prune(received_time, window_seconds);
  }

  void prune(const rclcpp::Time & now, double window_seconds)
  {
    while (!samples_.empty() && (now - samples_.front().received_time).seconds() > window_seconds) {
      samples_.pop_front();
    }
  }

  DelayStats calculate() const
  {
    DelayStats stats;
    stats.sample_count = samples_.size();
    if (samples_.empty()) {
      return stats;
    }

    stats.min = std::numeric_limits<double>::infinity();
    stats.max = -std::numeric_limits<double>::infinity();

    double sum = 0.0;
    for (const auto & sample : samples_) {
      sum += sample.delay_seconds;
      stats.min = std::min(stats.min, sample.delay_seconds);
      stats.max = std::max(stats.max, sample.delay_seconds);
    }
    stats.mean = sum / static_cast<double>(samples_.size());

    double variance = 0.0;
    for (const auto & sample : samples_) {
      const double delta = sample.delay_seconds - stats.mean;
      variance += delta * delta;
    }
    variance /= static_cast<double>(samples_.size());
    stats.stddev = std::sqrt(variance);
    return stats;
  }

  std::size_t size() const
  {
    return samples_.size();
  }

private:
  struct Sample
  {
    rclcpp::Time received_time;
    double delay_seconds{0.0};
  };

  std::deque<Sample> samples_;
};

}  // namespace mid360_deskew
