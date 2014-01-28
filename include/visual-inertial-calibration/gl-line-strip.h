// This file is part of the BA Project.
// Copyright (C) 2013 George Washington University,
//     Nima Keivan,
//     Gabe Sibley
//     Licensed under the Apache License, Version 2.0 (the "License");
//
// You may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
// implied.  See the License for the specific language governing
// permissions and limitations under the License.
#ifndef VISUAL_INERTIAL_CALIBRATION_GL_LINE_STRIP_H_
#define VISUAL_INERTIAL_CALIBRATION_GL_LINE_STRIP_H_

#include <pangolin/gl.h>
#include <vector>

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;

typedef std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> >
Vector6dAlignedVec;

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >
Vector3dAlignedVec;
}

class GLLineStrip {
 public:
  GLLineStrip()
      : buffer_(pangolin::GlArrayBuffer, 100, GL_FLOAT, 3, GL_DYNAMIC_DRAW) {
  }

  void Draw() {
    buffer_.Bind();
    glVertexPointer(buffer_.count_per_element, buffer_.datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_LINE_STRIP, buffer_.start(), buffer_.size());
    glPointSize(2.0);
    glDrawArrays(GL_POINTS, buffer_.start(), buffer_.size());
    glDisableClientState(GL_VERTEX_ARRAY);
    buffer_.Unbind();
  }

  void SetPoint(const Eigen::Vector3f& Point) {
    buffer_.Add(Point);
  }

  void SetPoint(double P[3]) {
    buffer_.Add(Eigen::Vector3f(P[0], P[1], P[2]));
  }

  void SetPoint(double X = 0, double Y = 0, double Z = 0) {
    buffer_.Add(Eigen::Vector3f(X, Y, Z));
  }

  void SetPoints(const std::vector<double>& vPts) {
    buffer_.Clear();
    for (size_t i = 0; i < vPts.size(); i += 3) {
      SetPoint(vPts[i], vPts[i + 1], vPts[i + 2]);
    }
  }

  void SetPointsFromTrajectory(const aligned_vector<Eigen::Vector6d>& vPts) {
    buffer_.Clear();
    for (size_t i = 0; i < vPts.size(); ++i) {
      SetPoint(vPts[i][0], vPts[i][1], vPts[i][2]);
    }
  }

  void SetPointsFromTrajectory(const aligned_vector<Eigen::Vector3d>& vPts) {
    buffer_.Clear();
    for (size_t i = 0; i < vPts.size(); ++i) {
      SetPoint(vPts[i][0], vPts[i][1], vPts[i][2]);
    }
  }

  void ClearLines() {
    buffer_.Clear();
  }

 private:
  pangolin::GlSizeableBuffer buffer_;
};

#endif  // VISUAL_INERTIAL_CALIBRATION_GL_LINE_STRIP_H_
