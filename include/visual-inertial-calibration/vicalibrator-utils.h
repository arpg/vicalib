/*
  This file is part of the BA Project.

  Copyright (C) 2013 George Washington University,
  Nima Keivan,
  Gabe Sibley

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#ifndef VISUAL_INERTIAL_CALIBRATION_VICALIBRATOR_UTILS_H_
#define VISUAL_INERTIAL_CALIBRATION_VICALIBRATOR_UTILS_H_

#include <sys/time.h>
#include <time.h>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <glog/logging.h>
#include <sophus/se3.hpp>


namespace visual_inertial_calibration {
// Short-hand for Eigen types
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

// #define ENABLE_TIMING
extern int debug_level_threshold;
extern int debug_level;

#ifdef ENABLE_TESTING
#define BA_TEST(x)  assert(x)
#else
#define BA_TEST(x)
#endif

static Eigen::IOFormat kCleanFmt(4, 0, ", ", ";\n", "", "");
static Eigen::IOFormat kLongFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "");
static const double kNormThreshold = 1e-3;
static const double kTestingEps = 1e-9;

// Transform a 4x1 (homogenous point) with an SE3.
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 4, 1> MultHomogeneous(
    const Sophus::SE3Group<Scalar>& lhs,
    const Eigen::Matrix<Scalar, 4, 1>& rhs) {
  Eigen::Matrix<Scalar, 4, 1> out;
  out.template head<3>() = lhs.so3()
      * (Eigen::Matrix<Scalar, 3, 1>) rhs.template head<3>()
      + lhs.translation() * rhs[3];

  out[3] = rhs[3];
  return out;
}

// Raises a floating point value to an integer exponent.
template<typename Scalar = double>
inline Scalar powi(const Scalar x, const int y) {
  if (y == 0) {
    return 1.0;
  } else if (y < 0) {
    return 1.0 / powi(x, -y);
  } else {
    Scalar ret = x;
    for (int ii = 1; ii < y; ++ii) {
      ret *= x;
    }
    return ret;
  }
}

// this function implements d vee(log(A * exp(x) * B) ) / dx,which is in R^{6x6}
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 6, 6> dLog_dX(const Sophus::SE3Group<Scalar>& a,
                                           const Sophus::SE3Group<Scalar>& b) {
  const Eigen::Matrix<Scalar, 6, 1> d_2 =
      Sophus::SE3Group<Scalar>::log(a * b) / 2;
  const Scalar d1 = d_2[3], d2 = d_2[4], d3 = d_2[5], dx = d_2[0], dy = d_2[1],
      dz = d_2[2];
  // this is using the 2nd order cbh expansion, to evaluate
  // (I + 0.5 [Adj*x, log(AB)])*Adj
  // refer to the thesis by Hauke Strasdat, Appendix 3.
  return (Eigen::Matrix<Scalar, 6, 6>() <<
          1, d3, -d2, 0, dz, -dy,
          -d3, 1, d1, -dz, 0, dx,
          d2, -d1, 1, dy, -dx, 0,
          0, 0, 0, 1, d3, -d2,
          0, 0, 0, -d3, 1, d1,
          0, 0, 0, d2, -d1, 1)
      .finished() * a.Adj();
}

// this function implements the derivative of log with respect to the input q
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 3, 4> dLog_dq(const Eigen::Quaternion<Scalar>& q) {
  const Scalar x = q.x();
  const Scalar y = q.y();
  const Scalar z = q.z();
  const Scalar w = q.w();
  const Scalar vec_squarednorm = powi(x, 2) + powi(y, 2) + powi(z, 2);
  const Scalar vec_norm = sqrt(vec_squarednorm);
  if (vec_norm < kTestingEps) {
    const Scalar s1 = 2 * vec_squarednorm;
    const Scalar s2 = 1.0 / powi(w, 3);
    const Scalar s3 = (3 * s1) / powi(w, 4) - 2 / powi(w, 2);
    const Scalar s4 = 2 / w;

    return (Eigen::Matrix<Scalar, 3, 4>()
            << -4 * s2 * powi(x, 2) + s4 - s1 * s2,
            -4 * x * y * s2,
            -4 * x * z * s2,
            x * s3,
            -4 * x * y * s2,
            -4 * s2 * powi(y, 2) + s4 - s1 * s2,
            -4 * y * z * s2,
            y * s3,
            -4 * x * z * s2,
            -4 * y * z * s2,
            -4 * s2 * powi(z, 2) + s4 - s1 * s2,
            z * s3).finished();

  } else {
    const Scalar s1 = vec_squarednorm;
    const Scalar s2 = 1 / (s1 / powi(w, 2) + 1);
    const Scalar s3 = atan(sqrt(s1) / w);
    const Scalar s4 = 1 / pow(s1, (3.0 / 2.0));
    const Scalar s5 = 1 / s1;
    const Scalar s6 = 1 / w;
    const Scalar s7 = (2 * s3) / sqrt(s1);
    const Scalar s8 = 2 * y * z * s2 * s5 * s6 - 2 * y * z * s3 * s4;
    const Scalar s9 = 2 * x * z * s2 * s5 * s6 - 2 * x * z * s3 * s4;
    const Scalar s10 = 2 * x * y * s2 * s5 * s6 - 2 * x * y * s3 * s4;

    return (Eigen::Matrix<Scalar, 3, 4>()
            << s7 - 2 * powi(x, 2) * s3 * s4 + 2 * powi(x, 2) * s2 * s5 * s6,
            s10, s9, -(2* x * s2) / powi(w, 2), s10,
            s7 - 2 * powi(y, 2) * s3 * s4 + 2 * powi(y, 2) * s2 * s5 * s6, s8,
            -(2 * y * s2) / powi(w, 2), s9, s8,
            s7 - 2 * powi(z, 2) * s3 * s4 + 2 * powi(z, 2) * s2 * s5 * s6,
            -(2 * z * s2) / powi(w, 2)).finished();
  }
}

template<typename Scalar = double>
inline std::vector<Eigen::Matrix<Scalar, 3, 3>,
                   Eigen::aligned_allocator<Eigen::Matrix<Scalar, 3, 3> > >
dLog_dR(const Eigen::Matrix<Scalar, 3, 3>& r) {
  std::vector<Eigen::Matrix<Scalar, 3, 3> > res(3);
  const Scalar s1 = r(0) / 2 + r(4) / 2 + r(8) / 2 - 0.5;
  const Scalar s2 = -(r(5) - r(7)) / (4 * (powi(s1, 2) - 1))
      - (s1 * acos(s1) * (r(5) - r(7))) / (4 * pow(1 - powi(s1, 2), 3.0 / 2.0));

  const Scalar s3 = acos(s1) / (2 * sqrt(1 - powi(s1, 2)));
  res[0] << s2, 0, 0, 0, s2, s3, 0, -s3, s2;

  const Scalar s4 = s1;  // r0/2 + R(4)/2 + R(8)/2 - 1/2
  const Scalar s5 = (r(2) - r(6)) / (4 * (powi(s4, 2) - 1))
      + (s4 * acos(s4) * (r(2) - r(6))) / (4 * pow(1 - powi(s4, 2), 3.0 / 2.0));

  const Scalar s6 = (1 / sqrt(1 - powi(s4, 2))) * acos(s4) * 0.5;
  res[1] << s5, 0, -s6, 0, s5, 0, s6, 0, s5;

  const Scalar s7 = s1;  // r0/2 + R(4)/2 + R(8)/2 - 1/2;
  const Scalar s8 = -(r(1) - r(3)) / (4 * (powi(s7, 2) - 1))
      - (s7 * acos(s7) * (r(1) - r(3))) / (4 * pow(1 - powi(s7, 2), 3.0 / 2.0));

  const Scalar s9 = acos(s7) / (2 * sqrt(1 - powi(s7, 2)));
  res[2] << s8, s9, 0, -s9, s8, 0, 0, 0, s8;

  return res;
}

// The 4x3 derivative of the SO3 exponential quaternion with respect
// to the tangent space parameters
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 4, 3> dqExp_dw(
    const Eigen::Matrix<Scalar, 3, 1>& w) {
  const Scalar t = w.norm();
  const Scalar s1 = t / 20 - 1;
  const Scalar s2 = powi(t, 2) / 48 - 0.5;
  const Scalar s3 = (s1 * w[1] * w[2]) / 24;
  const Scalar s4 = (s1 * w[0] * w[2]) / 24;
  const Scalar s5 = (s1 * w[0] * w[1]) / 24;
  const Scalar s6 = powi(t, 2);
  return (Eigen::Matrix<Scalar, 4, 3>()
          << (s1 * powi(w[0], 2)) / 24 - s6 / 48 + 0.5, s5, s4, s5,
          (s1 * powi(w[1], 2)) / 24 - s6 / 48 + 0.5, s3, s4, s3,
          (s1 * powi(w[2], 2)) / 24 - s6 / 48 + 0.5, (s2 * w[0]) / 2,
          (s2 * w[1]) / 2, (s2 * w[2]) / 2).finished();
}

// The 4x4 derivative of the quaternion inverse with respect to the
// quaternion parameters
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 4, 4> dqinv_dq() {
  return ((Eigen::Matrix<Scalar, 4, 1>() << 1, -1, -1, -1).finished())
      .asDiagonal();
}

// The 4x4 derivative of the multiplicative result of two quaternions
// q1 and q2 with respect to the parameters of q2
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 4, 4> dq1q2_dq2(
    const Eigen::Quaternion<Scalar>& q1) {
  return (Eigen::Matrix<Scalar, 4, 4>() << q1.w(), -q1.z(), q1.y(), q1.x(),
          q1.z(), q1.w(), -q1.x(), q1.y(), -q1.y(), q1.x(), q1.w(), q1.z(),
          -q1.x(), -q1.y(), -q1.z(), q1.w()).finished();
}

// The 4x4 derivative of the multiplicative result of two quaternions
// q1 and q2 with respect to the parameters of q1
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 4, 4> dq1q2_dq1(
    const Eigen::Quaternion<Scalar>& q2) {
  return (Eigen::Matrix<Scalar, 4, 4>() << q2.w(), q2.z(), -q2.y(), q2.x(),
          -q2.z(), q2.w(), q2.x(), q2.y(), q2.y(), -q2.x(), q2.w(), q2.z(),
          -q2.x(), -q2.y(), -q2.z(), q2.w()).finished();
}

// The 3x4 derivative of the 3d point x rotated by quaternion q with
// respect to the quaternion parameters
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 3, 4> dqx_dq(
    const Eigen::Quaternion<Scalar>& q,
    const Eigen::Matrix<Scalar, 3, 1>& vec) {
  const Scalar x = vec[0], y = vec[1], z = vec[2];
  const Scalar s1 = 2 * q.x() * y;
  const Scalar s2 = 2 * q.y() * y;
  const Scalar s3 = 2 * q.x() * x;
  const Scalar s4 = 2 * q.z() * x;
  const Scalar s5 = 2 * q.y() * z;
  const Scalar s6 = 2 * q.z() * z;

  return (Eigen::Matrix<Scalar, 3, 4>()
          << s2 + s6, s1 - 4 * q.y() * x + 2 * q.w() * z,
          2 * q.x() * z - 2 * q.w() * y - 4 * q.z() * x, s5 - 2 * q.z() * y,
          2 * q.y() * x - 4 * q.x() * y - 2 * q.w() * z, s3 + s6,
          s5 + 2 * q.w() * x - 4 * q.z() * y, s4 - 2 * q.x() * z,
          s4 + 2 * q.w() * y - 4 * q.x() * z,
          2 * q.z() * y - 2 * q.w() * x - 4 * q.y() * z, s2 + s3,
          s1 - 2 * q.y() * x).finished();
}

// The 7x7 derivative of the result of the multiplication of two
// transformations t1 and t2, with respect to the parameters of
// t1. The transformations are parameterized as a 7d vectors,
// comprising of 3 translation and 4 quaternion terms
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 7, 7> dt1t2_dt1(
    const Sophus::SE3Group<Scalar>& t1,
    const Sophus::SE3Group<Scalar>& t2) {

  Eigen::Matrix<Scalar, 7, 7> dt1t2_dt2;
  dt1t2_dt2.setZero();
  dt1t2_dt2.template block<3, 3>(0, 0).setIdentity();
  dt1t2_dt2.template block<3, 4>(0, 3) =
      dqx_dq(t1.unit_quaternion(), t2.translation());
  dt1t2_dt2.template block<4, 4>(3, 3) =
      dq1q2_dq1(t2.unit_quaternion());

  return dt1t2_dt2;
}

// this function implements d vee(log(A * exp(x) * B)) / dx,which is in R^{6x6}
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 6, 6> dLog_decoupled_dX(
    const Sophus::SE3Group<Scalar>& a, const Sophus::SE3Group<Scalar>& b) {
  const Eigen::Matrix<Scalar, 6, 1> d_2 = Sophus::SE3Group<Scalar>::log(
      a * b.inverse()) / 2;
  const Scalar d1 = d_2[3], d2 = d_2[4], d3 = d_2[5], dx = d_2[0], dy = d_2[1],
      dz = d_2[2];

  // this is using the 2nd order cbh expansion, to evaluate
  // (I + 0.5 [Adj*x, log(AB)])*Adj
  // refer to the thesis by Hauke Strasdat, Appendix 3.
  const Eigen::Matrix<Scalar, 6, 6> d_log(
      (Eigen::Matrix<Scalar, 6, 6>() <<
       1, d3, -d2, 0, dz, -dy,
       -d3, 1, d1, -dz, 0, dx,
       d2, -d1, 1, dy, -dx, 0,
       0, 0, 0, 1, d3, -d2,
       0, 0, 0, -d3, 1, d1,
       0, 0, 0, d2, -d1, 1)
      .finished() * a.Adj());

  Eigen::Matrix<Scalar, 6, 6> dLog_decoupled =
      Eigen::Matrix<Scalar, 6, 6>::Identity();
  dLog_decoupled.template block<3, 3>(3, 3) = d_log.template block<3, 3>(3, 3);
  return dLog_decoupled;
}

// The derivative of the SE3 logarithm (6d) with respect to a
// transformation (7d) which is parameterized as 3 translation and 4
// quaternion terms.
template<typename Scalar = double>
inline Eigen::Matrix<Scalar, 6, 7> dLog_dSE3(
    const Sophus::SE3Group<Scalar>& t) {
  const Eigen::Matrix<Scalar, 3, 4> dw_dq = dLog_dq(t.unit_quaternion());

  const Scalar x = t.translation()[0];
  const Scalar y = t.translation()[1];
  const Scalar z = t.translation()[2];
  const Eigen::Matrix<Scalar, 3, 1> w = t.so3().log();
  const Scalar wx = w[0];
  const Scalar wy = w[1];
  const Scalar wz = w[2];

  Eigen::Matrix<Scalar, 3, 1> upsilon_omega;
  Scalar theta;
  upsilon_omega.template tail<3>() =
      Sophus::SO3Group<Scalar>::logAndTheta(t.so3(), &theta);
  const Eigen::Matrix<Scalar, 3, 3>& omega =
      Sophus::SO3Group<Scalar>::hat(upsilon_omega.template tail<3>());

  // avoid division by 0
  bool close_to_zero =
      std::abs(theta) < Sophus::SophusConstants<Scalar>::epsilon();

  Eigen::Matrix<Scalar, 3, 3> v_inv;
  if (close_to_zero) {
    v_inv = Eigen::Matrix<Scalar, 3, 3>::Identity() -
        static_cast<Scalar>(0.5) * omega
        + static_cast<Scalar>(1./12.) * (omega * omega);
  } else {
    v_inv = (Eigen::Matrix<Scalar, 3, 3>::Identity()
             - static_cast<Scalar>(0.5) * omega
             + (static_cast<Scalar>(1)
                - theta / (static_cast<Scalar>(2) * tan(theta / Scalar(2))))
             / (theta * theta) * (omega * omega));
  }

  Eigen::Matrix<Scalar, 6, 7> dlog;
  dlog.setZero();
  dlog.template block<3, 3>(0, 0) = v_inv;
  dlog.template block<3, 4>(3, 3) = dw_dq;

  if (close_to_zero) {
    const Scalar div_12 = 1./12;
    const Scalar div_6  = 1./6.;
    const Scalar wx_x   = wx * x;
    const Scalar wy_x   = wy * x;
    const Scalar wz_x   = wz * x;
    const Scalar wx_y   = wx * y;
    const Scalar wy_y   = wy * y;
    const Scalar wz_y   = wz * y;
    const Scalar wx_z   = wx * z;
    const Scalar wy_z   = wy * z;
    const Scalar wz_z   = wz * z;

    const Eigen::Matrix<Scalar, 3, 3> dlog_dw(
        (Eigen::Matrix<Scalar, 3, 3>()
         << div_12 * (wy_y + wz_z),
         div_12 * wx_y - div_6 * wy_x - 0.5 * z,
         0.5 * y - div_6 * wz_x + div_12 * wx_z,
         0.5 * z + div_12 * wy_x - div_6 * wx_y,
         div_12 * (wx_x + wz_z),
         div_12 * wy_z - div_6 * wz_y - 0.5 * x,
         div_12 * wz_x - div_6 * wx_z - 0.5 * y,
         0.5 * x + div_12 * wz_y - div_6 * wy_z,
         div_12 * (wx_x * wy_y))
        .finished());

    dlog.template block<3, 4>(0, 3) = dlog_dw * dw_dq;

  } else {
    const Scalar s1 = powi(wx, 2) + powi(wy, 2) + powi(wz, 2);
    const Scalar s2 = tan(sqrt(s1) / 2);
    const Scalar s3 = sqrt(s1) / (2 * s2) - 1;
    const Scalar s4 = wz / (2 * sqrt(s1) * s2)
        - (wz * (powi(s2, 2) + 1)) / (4 * powi(s2, 2));
    const Scalar s5 = wy / (2 * sqrt(s1) * s2)
        - (wy * (powi(s2, 2) + 1)) / (4 * powi(s2, 2));
    const Scalar s6 = wx / (2 * sqrt(s1) * s2)
        - (wx * (powi(s2, 2) + 1)) / (4 * powi(s2, 2));
    const Scalar s7 = 1 / s1;
    const Scalar s8 = 1 / powi(s1, 2);
    const Scalar s9 = powi(wx, 2) + powi(wy, 2);
    const Scalar s10 = powi(wx, 2) + powi(wz, 2);
    const Scalar s11 = powi(wy, 2) + powi(wz, 2);
    const Scalar s12 = 2 * s3 * s8 * wx * wy * wz;
    const Scalar s13 = -2 * s3 * s8 * wy * powi(wz, 2) + s4 * s7 * wy * wz
        + s3 * s7 * wy;
    const Scalar s14 = -2 * s3 * s8 * wx * powi(wz, 2) + s4 * s7 * wx * wz
        + s3 * s7 * wx;
    const Scalar s15 = -2 * s3 * s8 * wz * powi(wy, 2) + s5 * s7 * wz * wy
        + s3 * s7 * wz;
    const Scalar s16 = -2 * s3 * s8 * wz * powi(wx, 2) + s6 * s7 * wz * wx
        + s3 * s7 * wz;
    const Scalar s17 = -2 * s3 * s8 * wx * powi(wy, 2) + s5 * s7 * wx * wy
        + s3 * s7 * wx;
    const Scalar s18 = -2 * s3 * s8 * wy * powi(wx, 2) + s6 * s7 * wy * wx
        + s3 * s7 * wy;
    const Scalar s19 = 2 * s3 * s7 * wy;
    const Scalar s20 = 2 * s3 * s7 * wx;

    const Eigen::Matrix<Scalar, 3, 3> dlog_dw(
        (Eigen::Matrix<Scalar, 3, 3>()
         << x * (s6 * s7 * s11 - 2 * s3 * s8 * s11 * wx) - s18 * y - s16 * z,
         x * (s19 + s5 * s7 * s11 - 2 * s3 * s8 * s11 * wy) - s17 * y
         - z * (s5 * s7 * wx * wz - 2 * s3 * s8 * wx * wy * wz + 0.5),

         x * (s4 * s7 * s11 + 2 * s3 * s7 * wz - 2 * s3 * s8 * s11 * wz)
         - s14 * z + y * (s12 - s4 * s7 * wx * wy + 0.5),

         y * (s20 + s6 * s7 * s10 - 2 * s3 * s8 * s10 * wx) - s18 * x
         + z * (s12 - s6 * s7 * wy * wz + 0.5),

         y * (s5 * s7 * s10 - 2 * s3 * s8 * s10 * wy) - s17 * x - s15 * z, y
         * (s4 * s7 * s10 + 2 * s3 * s7 * wz - 2 * s3 * s8 * s10 * wz)
         - s13 * z - x * (s4 * s7 * wx * wy - s12 + 0.5),

         z * (s20 + s6 * s7 * s9 - 2 * s3 * s8 * s9 * wx) - s16 * x
         - y * (s6 * s7 * wy * wz - s12 + 0.5), z
         * (s19 + s5 * s7 * s9 - 2 * s3 * s8 * s9 * wy) - s15 * y
         + x * (s12 - s5 * s7 * wx * wz + 0.5), z
         * (s4 * s7 * s9 - 2 * s3 * s8 * s9 * wz) - s14 * x - s13 * y)
        .finished());

    dlog.template block<3, 4>(0, 3) = dlog_dw * dw_dq;
  }
  return dlog;
}
}  // namespace visual_inertial_calibration
#endif  // VISUAL_INERTIAL_CALIBRATION_VICALIBRATOR_UTILS_H_
