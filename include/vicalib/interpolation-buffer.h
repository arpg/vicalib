// This file is part of the BA Project.
// Copyright (C) 2013 George Washington University,
// Nima Keivan,
// Gabe Sibley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISUAL_INERTIAL_CALIBRATION_INTERPOLATION_BUFFER_H_
#define VISUAL_INERTIAL_CALIBRATION_INTERPOLATION_BUFFER_H_

#include <algorithm>
#include <vector>
#include <vicalib/eigen-alignment.h>

namespace visual_inertial_calibration {

template <typename T>
inline double GetScalar(const T& v) {
  return v.a;
}

template <>
inline double GetScalar<double>(const double& v) {
  return v;
}

// Templated interpolation buffer. Used to smoothly interpolate
// between stored elements. The interplation is delimited by the time
// value.
//
// Scalar: the type used for the arithmetic (float or double)
//
// ElementType: The type of the element in the interpolation buffer. Needs to
// provide:
//
// ElementType& operator *(const Scalar rhs) : result of operation
// with a scalar
//
// ElementType& operator +(const ElementType& rhs) : result of
// addition with an element
template<template <class> class ElementT, typename Scalar>
struct InterpolationBufferT {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef ElementT<Scalar> ElementType;

  aligned_vector<ElementType> elements_;
  Scalar start_time_;
  Scalar end_time_;
  Scalar average_dt_;

  explicit InterpolationBufferT(unsigned int size)
      : start_time_(-1),
        end_time_(-1),
        average_dt_(0) {
    elements_.reserve(size);
  }

  // Adds an element to the interpolation buffer, updates the average
  // and end times
  void AddElement(const ElementType& element) {
    CHECK_GT(element.time, end_time_);
    const size_t nElems = elements_.size();
    Scalar dt = 0;
    if (nElems > 0) {
      dt = element.time - elements_.back().time;
    }

    // update the average dt
    average_dt_ = (average_dt_ * nElems + dt) / (nElems + 1);

    // add the element and update the end time
    elements_.push_back(element);
    end_time_ = element.time;
    start_time_ = elements_.front().time;
  }

  // Gets the next element in the buffer, depending on the maximum
  // time interval specified
  //
  // max_time: The maximum time interval. The returned element time
  // will be <= to this
  //
  // index_out: The output index of the returned element output The
  // output element
  //
  // Returns true if the function returned an intermediate element,
  // false if we had to interpolate in which case we've reached the
  // end
  template <typename T>
  bool GetNext(const Scalar& max_time, const T& dt, size_t* index_out,
               ElementT<T>* output) const {
    CHECK_NOTNULL(index_out);
    CHECK_NOTNULL(output);

    // if we have reached the end, interpolate and signal the end
    if (*index_out + 1 >= elements_.size()) {
      *output = GetElement(max_time, dt, index_out);
      return false;
    } else if (elements_[*index_out + 1].time + dt > static_cast<T>(max_time)) {
      *output = GetElement(max_time, dt, index_out);
      return false;
    } else {
      *output = elements_[++*index_out];
      output->time = output->time + dt;
      return true;
    }
  }

  // Returns whether or not an element exists for the given time
  // time: The time for which we check the element
  template <typename T>
  bool HasElement(const Scalar time, const T& dt) const {
    return (time >= start_time_ + GetScalar(dt) &&
            time <= end_time_ + GetScalar(dt));
  }

  // Returns an interpolated element. Call HasElement before this
  // function to make sure an element exists for this time
  ElementType GetElement(const Scalar time) const {
    size_t index;
    return GetElement(time, 0., &index);
  }

  // Interpolate from a to b by the amount given in fraction of type T
  template <typename T>
  static void Interpolate(const ElementType& a, const ElementType& b,
                          const T& t_a, const T& t_b, const T& t_out,
                          ElementT<T>* out) {
    CHECK_NOTNULL(out);
    T fraction = (t_out - t_a) / (t_b - t_a);
    *out = (static_cast<ElementT<T> >(a) * (static_cast<T>(1.0) - fraction) +
            static_cast<ElementT<T> >(b) * fraction);
    out->time = t_out;
  }

  template <typename T>
  void InterpolateElements(size_t a_index, size_t b_index,
                           const T& time_offset, Scalar time,
                           ElementT<T>* out) const {
    Interpolate(elements_[a_index], elements_[b_index],
                elements_[a_index].time + time_offset,
                elements_[b_index].time + time_offset,
                static_cast<T>(time),
                out);
  }

  // Returns an interpolated element. Call HasElement before this
  //  function to make sure an element exists for this time
  template <typename T>
  ElementT<T> GetElement(const Scalar time, const T& dt, size_t* pIndex) const {
    CHECK_NOTNULL(pIndex);

    Scalar imu_ts_offset = GetScalar(dt);

    // guess what the index would be
    size_t guess_idx = (time - start_time_ + imu_ts_offset) / average_dt_;
    const size_t n_elements = elements_.size();
    guess_idx = std::min(std::max(guess_idx, static_cast<size_t>(0u)),
                         n_elements - static_cast<size_t>(1u));
    CHECK_GE(guess_idx, 0);

    // now using the guess, find a direction to go
    ElementT<T> result;
    if (elements_[guess_idx].time + imu_ts_offset > time) {
      // we need to go backwards
      if (guess_idx == 0) {
        result = elements_.front();
        result.time = result.time + dt;
        *pIndex = guess_idx;
      } else {
        while ((guess_idx - 1) > 0 &&
               elements_[guess_idx - 1].time + imu_ts_offset > time) {
          --guess_idx;
        }
        InterpolateElements(guess_idx - 1, guess_idx, dt, time, &result);
        *pIndex = guess_idx - 1;
      }
    } else {
      // we need to go forwards
      if (guess_idx == n_elements - 1) {
        *pIndex = guess_idx;
        result = elements_.back();
        result.time = result.time + dt;
      } else {
        while ((guess_idx + 1) < n_elements &&
               (elements_[guess_idx + 1].time + imu_ts_offset) < time) {
          ++guess_idx;
        }
        InterpolateElements(guess_idx, guess_idx + 1, dt, time, &result);
        *pIndex = guess_idx;
      }
    }
    return result;
  }

// Retrieve measurements between two times (inclusive).
  template <typename T>
  void GetRange(const Scalar start_time,
                const Scalar end_time,
                const T& dt,
                aligned_vector<ElementT<T> >* measurements) const {
    CHECK_NOTNULL(measurements);
    size_t index;

    // get all the imu measurements between these two poses, and add
    // them to a vector
    if (HasElement(start_time, dt)) {
      measurements->push_back(GetElement(start_time, dt, &index));
      ElementT<T> meas;
      while (GetNext(end_time, dt, &index, &meas)) {
        measurements->push_back(meas);
      }

      measurements->push_back(meas);
    }
  }
};

}  // namespace visual_inertial_calibration

#endif  // VISUAL_INERTIAL_CALIBRATION_INTERPOLATION_BUFFER_H_
