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
  ElementVec elements;
  Scalar start_time;
  Scalar end_time;
  Scalar average_dt;

  explicit InterpolationBufferT(unsigned int size = 1000)
      : start_time(-1),
        end_time(-1) {
    elements.reserve(size);
  }

  // Adds an element to the interpolation buffer, updates the average
  // and end times
  void AddElement(const ElementType& element) {
    CHECK_GT(element.time, end_time);
    const size_t nElems = elements.size();
    const Scalar dt = nElems == 0 ? 0 : element.time - elements.back().time;

    // update the average dt
    average_dt = (average_dt * nElems + dt) / (nElems + 1);

    // add the element and update the end time
    elements.push_back(element);
    end_time = element.time;
    start_time = elements.front().time;
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
    if (*index_out + 1 >= elements.size()) {
      *output = GetElement(max_time, index_out);
      return false;
    } else if (elements[*index_out + 1].time > max_time) {
      *output = GetElement(max_time, index_out);
      return false;
    } else {
      ++(*index_out);
      *output = elements[*index_out];
      return true;
    }
  }

  // Returns whether or not an element exists for the given time
  // time: The time for which we check the element
  bool HasElement(const Scalar time) {
    return (time >= start_time && time <= end_time);
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
    size_t guess_idx = (time - start_time) / average_dt;
    const size_t n_elements = elements.size();
    guess_idx = std::min(std::max(static_cast<unsigned int>(guess_idx), 0u),
                         static_cast<unsigned int>(elements.size()) - 1u);

    // now using the guess, find a direction to go
    if (elements[guess_idx].time > time) {
      // we need to go backwards
      if (guess_idx == 0) {
        *pIndex = guess_idx;
        return elements.front();
      }

      while ((guess_idx - 1) > 0 && elements[guess_idx - 1].time > time) {
        --guess_idx;
      }
      const Scalar interpolator = ((time - elements[guess_idx - 1].time) /
                                   (elements[guess_idx].time -
                                    elements[guess_idx - 1].time));

      *pIndex = guess_idx - 1;
      ElementType res = (elements[guess_idx - 1] * (1 - interpolator) +
                         elements[guess_idx] * interpolator);

      res.time = time;
      return res;
    }

    // we need to go forwards
    if (guess_idx == n_elements - 1) {
      *pIndex = guess_idx;
      return elements.back();
    }

    while ((guess_idx + 1) < n_elements &&
           elements[guess_idx + 1].time < time) {
      ++guess_idx;
    }

    const Scalar interpolator = ((time - elements[guess_idx].time) /
                                 (elements[guess_idx + 1].time -
                                  elements[guess_idx].time));

    *pIndex = guess_idx;
    ElementType res = (elements[guess_idx] * (1 - interpolator) +
                       elements[guess_idx + 1] * interpolator);
    res.time = time;
    return res;
  }

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
