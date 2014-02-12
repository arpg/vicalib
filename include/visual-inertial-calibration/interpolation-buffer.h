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

namespace visual_inertial_calibration {

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
template<typename ElementType, typename Scalar>
struct InterpolationBufferT {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::vector<ElementType,
                      Eigen::aligned_allocator<ElementType> > ElementVec;
  ElementVec elements_;
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
  bool GetNext(const Scalar max_time, size_t* index_out,
               ElementType* output) {
    CHECK_NOTNULL(index_out);
    CHECK_NOTNULL(output);

    // if we have reached the end, interpolate and signal the end
    if (*index_out + 1 >= elements_.size()) {
      *output = GetElement(max_time, index_out);
      return false;
    } else if (elements_[*index_out + 1].time > max_time) {
      *output = GetElement(max_time, index_out);
      return false;
    } else {
      ++(*index_out);
      *output = elements_[*index_out];
      return true;
    }
  }

  // Returns whether or not an element exists for the given time
  // time: The time for which we check the element
  bool HasElement(const Scalar time) {
    return (time >= start_time_ && time <= end_time_);
  }

  // Returns an interpolated element. Call HasElement before this
  // function to make sure an element exists for this time
  ElementType GetElement(const Scalar time) {
    size_t index;
    return GetElement(time, &index);
  }

  // Returns an interpolated element. Call HasElement before this
  //  function to make sure an element exists for this time
  ElementType GetElement(const Scalar time, size_t* pIndex) {
    CHECK_NOTNULL(pIndex);

    // guess what the index would be
    size_t guess_idx = (time - start_time_) / average_dt_;
    const size_t n_elements = elements_.size();
    guess_idx = std::min(std::max(static_cast<unsigned int>(guess_idx), 0u),
                         static_cast<unsigned int>(elements_.size()) - 1u);
    CHECK_GE(guess_idx, 0);

    // now using the guess, find a direction to go
    if (elements_[guess_idx].time > time) {
      // we need to go backwards
      if (guess_idx == 0) {
        *pIndex = guess_idx;
        return elements_.front();
      }

      while ((guess_idx - 1) > 0 && elements_[guess_idx - 1].time > time) {
        --guess_idx;
      }
      const Scalar interpolator = ((time - elements_[guess_idx - 1].time) /
                                   (elements_[guess_idx].time -
                                    elements_[guess_idx - 1].time));

      *pIndex = guess_idx - 1;
      ElementType res = (elements_[guess_idx - 1] * (1 - interpolator) +
                         elements_[guess_idx] * interpolator);

      res.time = time;
      return res;
    }

    // we need to go forwards
    if (guess_idx == n_elements - 1) {
      *pIndex = guess_idx;
      return elements_.back();
    }

    while ((guess_idx + 1) < n_elements &&
           elements_[guess_idx + 1].time < time) {
      ++guess_idx;
    }

    const Scalar interpolator = ((time - elements_[guess_idx].time) /
                                 (elements_[guess_idx + 1].time -
                                  elements_[guess_idx].time));

    *pIndex = guess_idx;
    ElementType res = (elements_[guess_idx] * (1 - interpolator) +
                       elements_[guess_idx + 1] * interpolator);
    res.time = time;
    return res;
  }

// Retrieve measurements between two times (inclusive).
  void GetRange(const Scalar start_time,
                const Scalar end_time,
                ElementVec* measurements) {
    CHECK_NOTNULL(measurements);
    size_t index;

    // get all the imu measurements between these two poses, and add
    // them to a vector
    if (HasElement(start_time)) {
      measurements->push_back(GetElement(start_time, &index));
      ElementType meas;
      while (GetNext(end_time, &index, &meas)) {
        measurements->push_back(meas);
      }
      measurements->push_back(meas);
    }
  }
};

}  // namespace visual_inertial_calibration

#endif  // VISUAL_INERTIAL_CALIBRATION_INTERPOLATION_BUFFER_H_
