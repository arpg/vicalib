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

#ifndef VISUAL_INERTIAL_CALIBRATION_CERES_COST_FUNCTIONS_H_
#define VISUAL_INERTIAL_CALIBRATION_CERES_COST_FUNCTIONS_H_

#include <vector>
#include <Eigen/StdVector>
#include <visual-inertial-calibration/types.h>

namespace visual_inertial_calibration {

// Integrates a given pose and velocity using Euler integration, given
// the derivative k and duration dt
//
//     pose - The structure specifying the starting position,
//     rotation, velocity and timestamp
//
//     k - The derivative vector comprising of 3 velocity, 3 SO3
//     tangent space and 3 acceleration parameters, used to integrate
//     the position, quaternion and velocity respectively.
//
//     dt - The duration of the Euler integration
template<typename T>
static ImuPoseT<T> IntegratePoseJet(const ImuPoseT<T>& pose,
                                    const Eigen::Matrix<T, 9, 1>& k,
                                    const T& dt) {
  const Sophus::SO3Group<T> rv2_v1(
      Sophus::SO3Group<T>::exp(k.template segment<3>(3) * static_cast<T>(dt)));

  ImuPoseT<T> y = pose;
  y.t_wp_.translation() += k.template head<3>() * static_cast<T>(dt);
  const Eigen::Matrix<T, 4, 1> temp =
      (rv2_v1.unit_quaternion() * pose.t_wp_.so3().unit_quaternion()).coeffs();
  for (int i = 0; i < 4; ++i) {
      y.t_wp_.so3().data()[i] = temp.data()[i];
  }

  // Do euler integration for now.
  y.v_w_ += k.template tail<3>() * static_cast<T>(dt);
  return y;
}

// Returns the 9d derivative vector used in the IntegratePoseJet
// function, evaluated at a specific offset between the timestamps of
// z_start and z_end
//
//     pose - The structure specifying the position, rotation and
//     velocity
//
//     t_w - The 3d gravity vector
//
//     z_start - The accelerometer and gyroscope readings at the start
//     of the integration window (contains timestamp)
//
//     z_end - The accelerometer and gyroscope readings at the end of
//     the integration window (contains timestamp)
//
//     bg - 3d vector containing gyroscope biases
//
//     ba - 3d vector containing accelerometer biases
//
//     dt - The time offset between z_start and z_end at which the
//     derivative is evaluated
template<typename T>
static Eigen::Matrix<T, 9, 1> GetPoseDerivativeJet(
    const ImuPoseT<T>& pose, const Eigen::Matrix<T, 3, 1>& t_w,
    const ImuMeasurementT<T>& z_start,
    const ImuMeasurementT<T>& z_end,
    const Eigen::Matrix<T, 3, 1>& bg, const Eigen::Matrix<T, 3, 1>& ba,
    const Eigen::Matrix<T, 6, 1>& sf, const T dt) {
  T alpha = (z_end.time - (z_start.time + dt)) / (z_end.time - z_start.time);
  Eigen::Matrix<T, 3, 1> zg = z_start.w_ * alpha
      + z_end.w_ * (1.0 - alpha);
  Eigen::Matrix<T, 3, 1> za = z_start.a_ * alpha
      + z_end.a_ * (1.0 - alpha);

  Eigen::Matrix<T, 9, 1> deriv;
  // Derivative of position is velocity.
  // v (velocity)
  deriv.template head<3>() = pose.v_w_;

  // w (angular rates)
  deriv.template segment<3> (3) = pose.t_wp_.so3().Adj()
      * (zg.template cast<T>().cwiseProduct(sf.template head<3>()) + bg);
  // a (acceleration)
  deriv.template segment<3> (6) = pose.t_wp_.so3()
      * (za.template cast<T>().cwiseProduct(sf.template tail<3>()) + ba) - t_w;

  return deriv;
}

// Given the start and end imu measurements (which include
// timestamps), this function integrates the position, rotation and
// velocities specified in 'pose', using RK45 integration
//
//     pose - The structure specifying the starting position,
//     rotation, velocity and timestamp
//
//     z_start - The accelerometer and gyroscope readings at the start
//     of the integration window (contains timestamp)
//
//     z_end - The accelerometer and gyroscope readings at the end of
//     the integration window (contains timestamp)
//
//     bg - 3d vector containing gyroscope biases
//
//     ba - 3d vector containing accelerometer biases
//
//     sf - 6d vector specifying scale factors for the gyroscope and
//     accelerometer
//
//     g - 3d vector specifying the gravity vector
//
//     dy_db - The 10x6 jacobian of the final pose and velocity, with
//     respect to the biases. The final pose is parameterized as a 10d
//     vector comprising of 3 translation, 4 quaternion and 3 velocity
//     parameters.
//
//     dy_dy0 - The 10x10 jacobian of the final pose and velocity as a
//     function of the initial pose and velocity. Both the initial and
//     final poses are parameterized as a 10d vector comprising of 3
//     translation, 4 quaternion and 3 velocity parameters.
template<typename T>
static ImuPoseT<T> IntegrateImuJet(
    const ImuPoseT<T>& pose,
    const ImuMeasurementT<T>& z_start,
    const ImuMeasurementT<T>& z_end,
    const Eigen::Matrix<T, 3, 1>& bg,
    const Eigen::Matrix<T, 3, 1>& ba,
    const Eigen::Matrix<T, 6, 1>& sf,
    const Eigen::Matrix<T, 3, 1>& g,
    Eigen::Matrix<T, 10, 6>* /* dy_db */ = 0,
    Eigen::Matrix<T, 10, 10>* /*dy_dy0*/ = 0) {
  // construct the state matrix
  if (z_end.time == z_start.time) {
    return pose;
  }

  T dt = z_end.time - z_start.time;

  Eigen::Matrix<T, 9, 1> k;
  const Eigen::Matrix<T, 9, 1> k1 = GetPoseDerivativeJet<T>(
      pose, g, z_start, z_end, bg, ba, sf, static_cast<T>(0));
  const ImuPoseT<T> y1 = IntegratePoseJet<T>(
      pose, k1, dt * static_cast<T>(0.5));
  const Eigen::Matrix<T, 9, 1> k2 = GetPoseDerivativeJet<T>(
      y1, g, z_start, z_end, bg, ba, sf, dt / static_cast<T>(2));
  const ImuPoseT<T> y2 = IntegratePoseJet<T>(pose, k2,
                                             dt * static_cast<T>(0.5));
  const Eigen::Matrix<T, 9, 1> k3 = GetPoseDerivativeJet<T>(
      y2, g, z_start, z_end, bg, ba, sf, dt / static_cast<T>(2));
  const ImuPoseT<T> y3 = IntegratePoseJet<T>(pose, k3, dt);
  const Eigen::Matrix<T, 9, 1> k4 = GetPoseDerivativeJet<T>(
      y3, g, z_start, z_end, bg, ba, sf, dt);

  k = (k1 + static_cast<T>(2) * k2 + static_cast<T>(2) * k3 + k4);
  ImuPoseT<T> res = IntegratePoseJet<T>(pose, k, dt / static_cast<T>(6.0));

  res.w_w_ = k.template segment<3> (3);
  res.time_ = static_cast<T>(z_end.time);
  return res;
}

// Integrates the measurements given in the measurement vector,
// considering the biases, scale factor, gravity vector and starting
// pose
//
//     pose - The structure specifying the starting position,
//     rotation, velocity and timestamp
//
//     measurements - The vector specifying all measurements and their
//     timestamps used in the integration
//
//     bg - 3d vector containing gyroscope biases
//
//     ba - 3d vector containing accelerometer biases
//
//     sf - 6d vector specifying scale factors for the gyroscope and
//     accelerometer
//
//     g - 3d vector specifying the gravity vector
//
//     poses - vector of intermediate poses output by the function
template<typename T>
static ImuPoseT<T> IntegrateResidualJet(
    const PoseT<T>& pose,
    const aligned_vector<ImuMeasurementT<T> >& measurements,
    const Eigen::Matrix<T, 3, 1>& bg,
    const Eigen::Matrix<T, 3, 1>& ba,
    const Eigen::Matrix<T, 6, 1>& sf,
    const Eigen::Matrix<T, 3, 1>& g,
    std::vector<ImuPoseT<T>, Eigen::aligned_allocator<ImuPoseT<T> > >* poses) {
  ImuPoseT<T> imu_pose(pose.t_wp_, pose.v_w_, Eigen::Matrix<T, 3, 1>::Zero(),
                       pose.time_);

  const ImuMeasurementT<T>* prev_meas = 0;
  poses->clear();
  poses->reserve(measurements.size() + 1);
  poses->push_back(imu_pose);

  // integrate forward in time, and retain all the poses
  for (const ImuMeasurementT<T>& meas : measurements) {
    if (prev_meas != NULL) {
      imu_pose = IntegrateImuJet<T>(imu_pose, *prev_meas, meas,
                                            bg, ba, sf, g);
      poses->push_back(imu_pose);
    }
    prev_meas = &meas;
  }

  return imu_pose;
}

/// Ceres autodifferentiatable cost function for pose errors.
/// The parameters are error state and should be a 6d pose delta
template<typename Scalar = double>
struct GlobalPoseCostFunction {
  GlobalPoseCostFunction(const Sophus::SE3Group<Scalar>& measurement,
                         const double weight = 1.0)
      : t_wc(measurement),
        weight(weight) {
  }

  template<typename T>
  bool operator()(const T* const t_t_ic, const T* const t_t_wi,
                  T* residuals) const {
    CHECK_NOTNULL(t_t_ic);
    CHECK_NOTNULL(t_t_wi);
    CHECK_NOTNULL(residuals);

    const Eigen::Map<const Sophus::SE3Group<T> > t_ic(t_t_ic);
    const Eigen::Map<const Sophus::SE3Group<T> > t_wi(t_t_wi);
    // the pose residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);

    pose_residuals = (t_wi * t_ic * t_wc.inverse().template cast<T>()).log()
        * static_cast<T>(weight);

    pose_residuals.tail(3) * static_cast<T>(weight);
    pose_residuals.head(3) * static_cast<T>(weight);
    return true;
  }

  const Sophus::SE3Group<Scalar> t_wc;
  const double weight;
};

template<typename Scalar = double>
struct FullImuCostFunction {
  FullImuCostFunction(
      const std::vector<ImuMeasurementT<Scalar>,
      Eigen::aligned_allocator<ImuMeasurementT<Scalar> > >& meas,
      const double w)
      : measurements(meas),
        weight(w) {}

  template<typename T>
  bool operator()(const T* const _tx2, const T* const _tx1,
                  const T* const _tvx2, const T* const _tvx1,
                  const T* const _tg, const T* const _tbg, const T* const _tba,
                  T* residuals) const {
    CHECK_NOTNULL(_tx2);
    CHECK_NOTNULL(_tx1);
    CHECK_NOTNULL(_tvx2);
    CHECK_NOTNULL(_tvx1);
    CHECK_NOTNULL(_tg);
    CHECK_NOTNULL(_tbg);
    CHECK_NOTNULL(_tba);
    CHECK_NOTNULL(residuals);

    // the residual vector consists of a 6d pose and a 3d velocity residual
    // the pose residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);
    // the velocity residuals
    Eigen::Map<Eigen::Matrix<T, 3, 1> > vel_residuals(&residuals[6]);

    // parameter vector consists of a
    // 6d pose delta plus starting
    // velocity and 2d gravity angles
    const Eigen::Map<const Sophus::SE3Group<T> > t_wx2(_tx2);
    const Eigen::Map<const Sophus::SE3Group<T> > t_wx1(_tx1);
    const Eigen::Map<const Sophus::SO3Group<T> > R_wx1(&_tx1[0]);

    // the velocity at the starting point
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > v1(_tvx1);

    // the velocity at the end point
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > v2(_tvx2);

    // gyro bias
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > bg(_tbg);

    // accelerometer bias
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > ba(_tba);

    // the 2d gravity vector (angles)
    const Eigen::Map<const Eigen::Matrix<T, 2, 1> > g(_tg);

    // get the gravity components in 3d based on
    // the 2 angles of the gravity vector
    const Eigen::Matrix<T, 3, 1> g_vector = GetGravityVector<T>(g, gravity());

    PoseT<T> start_pose;
    start_pose.t_wp_ = t_wx1;
    start_pose.v_w_ = v1;
    start_pose.time_ = measurements.front().Time;
    std::vector<ImuPoseT<T>, Eigen::aligned_allocator<ImuPoseT<T> > > vPoses;
    ImuPoseT<T> end_pose = IntegrateResidualJet<T>(start_pose,
                                                           measurements, bg, ba,
                                                           g_vector, &vPoses);

    // and now calculate the error with this pose
    pose_residuals = (end_pose.t_wp_ * t_wx2.inverse()).log() *
        static_cast<T>(weight);

    // to calculate the velocity error, first augment the IMU integration
    // velocity with gravity and initial velocity
    vel_residuals = (end_pose.v_w_ - v2) * static_cast<T>(weight) *
        static_cast<T>(0.1);
    return true;
  }

  const aligned_vector<ImuMeasurementT<Scalar> > measurements;
  const double weight;
};

template<typename ProjModel>
struct ImuReprojectionCostFunctor {
  ImuReprojectionCostFunctor(const Eigen::Vector3d& pw,
                             const Eigen::Vector2d& pc)
      : p_w(pw),
        p_c(pc) {
  }

  template<typename T>
  bool operator()(const T* const _t_wk, const T* const _r_ck,
                  const T* const _t_ck, const T* const cam_params,
                  T* residuals) const {
    CHECK_NOTNULL(_t_wk);
    CHECK_NOTNULL(_r_ck);
    CHECK_NOTNULL(_t_ck);
    CHECK_NOTNULL(cam_params);
    CHECK_NOTNULL(residuals);

    Eigen::Map<Eigen::Matrix<T, 2, 1> > r(residuals);
    const Eigen::Map<const Sophus::SE3Group<T> > t_wk(_t_wk);
    const Sophus::SE3Group<T> t_kw = t_wk.inverse();
    const Eigen::Map<const Sophus::SO3Group<T> > R_ck(_r_ck);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_ck(_t_ck);
    const Sophus::SE3Group<T> t_ck(R_ck, p_ck);

    const Eigen::Matrix<T, 3, 1> p_cv = (t_ck * (t_kw * p_w.cast<T>()));
    const Eigen::Matrix<T, 2, 1> z = ProjModel::template Project<T>(p_cv,
                                                                    cam_params);
    r = z - p_c.cast<T>();

    return true;
  }

  Eigen::Vector3d p_w;
  Eigen::Vector2d p_c;
};

template<typename Scalar = double>
struct SwitchedFullImuCostFunction {
  SwitchedFullImuCostFunction(
      const InterpolationBufferT<ImuMeasurementT, Scalar>* imu_buffer,
      Scalar start_time, Scalar end_time,
      const Eigen::Matrix<Scalar, 9, 9>& w_sqrt,
      const bool* res_switch)
      : start_time_(start_time), end_time_(end_time),
        imu_buffer_(imu_buffer),
        weight_sqrt_(w_sqrt),
        residal_switch_(res_switch) {}

  /** Get the measurements with a given time offset */
  template <typename T>
  void GetMeasurements(
      const T& time_offset,
      aligned_vector<ImuMeasurementT<T> >* measurements) const {
    imu_buffer_->GetRange(start_time_,
                          end_time_,
                          time_offset,
                          measurements);
  }

  template<typename T>
  bool operator()(const T* const _tx2, const T* const _tx1,
                  const T* const _tvx2, const T* const _tvx1,
                  const T* const _tg, const T* const _tb,
                  const T* const _tsf, const T* const _ttime_offset,
                  T* residuals) const {
    CHECK_NOTNULL(_tx2);
    CHECK_NOTNULL(_tx1);
    CHECK_NOTNULL(_tvx2);
    CHECK_NOTNULL(_tvx1);
    CHECK_NOTNULL(_tg);
    CHECK_NOTNULL(_tb);
    CHECK_NOTNULL(_tsf);
    CHECK_NOTNULL(_ttime_offset);
    CHECK_NOTNULL(residuals);
    // the residual vector consists of a 6d pose and a 3d velocity residual
    Eigen::Map<Eigen::Matrix<T, 9, 1> > residuals_vec(residuals);

    // parameter vector consists of a 6d pose delta plus starting velocity
    // and 2d gravity angles
    const Eigen::Map<const Sophus::SE3Group<T> > t_wx2(_tx2);
    const Eigen::Map<const Sophus::SE3Group<T> > t_wx1(_tx1);
    const Eigen::Map<const Sophus::SO3Group<T> > r_wx1(&_tx1[0]);

    // the velocity at the starting point
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > v1(_tvx1);

    // the velocity at the end point
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > v2(_tvx2);

    // gyro bias
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > bg(_tb);

    // accelerometer bias
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > ba(&_tb[3]);

    // gyro/accel scale factor
    const Eigen::Map<const Eigen::Matrix<T, 6, 1> > sf(_tsf);

    // the 2d gravity vector (angles)
    const Eigen::Map<const Eigen::Matrix<T, 2, 1> > g(_tg);

    // get the gravity components in 3d based on the 2
    // angles of the gravity vector
    const Eigen::Matrix<T, 3, 1> g_vector =
        GetGravityVector<T>(g, static_cast<T>(gravity()));

    aligned_vector<ImuMeasurementT<T> > measurements;
    GetMeasurements(*_ttime_offset, &measurements);

    CHECK_GT(measurements.size(), 0)
        << "IMU measurements required for cost function calculation.";

    PoseT<T> start_pose;
    start_pose.t_wp_ = t_wx1;
    start_pose.v_w_ = v1;
    start_pose.time_ = measurements.front().time;
    std::vector<ImuPoseT<T>, Eigen::aligned_allocator<ImuPoseT<T> > > poses;
    ImuPoseT<T> end_pose = IntegrateResidualJet<T>(
        start_pose, measurements, bg, ba, sf, g_vector, &poses);
    // and now calculate the error with this pose
    residuals_vec.template head<6>() = (end_pose.t_wp_ * t_wx2.inverse()).log();

    // to calculate the velocity error, first augment the IMU integration
    // velocity with gravity and initial velocity
    residuals_vec.template tail<3>() = (end_pose.v_w_ - v2);

    // Multiply by the weight matrix, which has been square rooted to be in
    // standard.
    residuals_vec = (residuals_vec.transpose() *
                     weight_sqrt_.template cast<T>()).transpose();

    if (*residal_switch_) {
      residuals_vec.template head<3>().setZero();
      residuals_vec.template tail<3>().setZero();
    }
    return true;
  }

  Scalar start_time_, end_time_;
  const InterpolationBufferT<ImuMeasurementT, Scalar>* imu_buffer_;
  Eigen::Matrix<Scalar, 9, 9> weight_sqrt_;
  const bool* residal_switch_;
};

}  // namespace visual_inertial_calibration

#endif  // VISUAL_INERTIAL_CALIBRATION_CERES_COST_FUNCTIONS_H_
