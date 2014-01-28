// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <queue>
#include <visual-inertial-calibration/eigen-alignment.h>

/** A moving average (boxcar) filter for smoothing sampled data and
 * detecting if it is at a stable value. */
template <typename DataT>
class BoxcarFilter {
 public:
  /**
   * Construct a boxcar filter.
   *
   * @param window_size The size of the moving average window
   *
   * @param stability_threshold The maximum change between two samples
   *                            to be considered "stable"
   */
  BoxcarFilter(size_t window_size, double stability_threshold)
      : current_sum_(DataT::Zero()), current_mean_(DataT::Zero()),
        stability_threshold_(stability_threshold),
        window_size_(window_size), stable_(false) {}
  virtual ~BoxcarFilter() {}

  /**
   * Add a new sample to the filter.
   *
   * Updates internal average and stability metrics.
   */
  void Add(const DataT& sample) {
    if (buffer_.size() == window_size_) {
      current_sum_ -= buffer_.front();
      buffer_.pop();
    }

    auto abs_sample = sample.cwiseAbs();
    stable_ = (abs_sample - current_mean_).norm() < stability_threshold_;
    current_sum_ += abs_sample;
    buffer_.push(abs_sample);
    current_mean_ = current_sum_ / buffer_.size();
  }

  /**
   * Has the data reached a consistently stable value.
   */
  bool IsStable() const {
    return buffer_.size() == window_size_ && stable_;
  }

  /**
   * The moving average of the samples
   */
  double CurrentValue() const {
    return current_mean_;
  }

  void set_stability_threshold(double stability_threshold) {
    stability_threshold_ = stability_threshold;
  }

  double stability_threshold() const {
    return stability_threshold_;
  }

  /** Set the size of the boxcar window */
  void set_size(size_t size) {
    window_size_ = size;
  }

  size_t size() const {
    return window_size_;
  }

 private:
  std::queue<DataT, aligned<std::deque, DataT> > buffer_;
  DataT current_sum_, current_mean_;
  double stability_threshold_;
  size_t window_size_;
  bool stable_;
};
