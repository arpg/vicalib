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

#ifndef VISUAL_INERTIAL_CALIBRATION_TYPES_H_
#define VISUAL_INERTIAL_CALIBRATION_TYPES_H_

#include <vector>

#include <calibu/Calibu.h>
#include <Eigen/Eigen>
#include <visual-inertial-calibration/eigen-alignment.h>
#include <sophus/se3.hpp>

#include <visual-inertial-calibration/vicalibrator-utils.h>

// TODO(renzo): the values below should come from calibration-provider
#define IMU_GYRO_UNCERTAINTY 7.15584993e-5  // 0.00104719755 // 0.1 //
#define IMU_ACCEL_UNCERTAINTY 0.00159855109  // 0.0392266 // 10

namespace visual_inertial_calibration {

static const double kGravity = 9.8007;  // m/s^2

template<typename Scalar = double>
struct PoseT {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Sophus::SE3Group<Scalar> t_wp;
  Sophus::SE3Group<Scalar> t_vs;
  Eigen::Matrix<Scalar, 3, 1> v_w;
  Eigen::Matrix<Scalar, 6, 1> b;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> cam_params;
  std::vector<bool> param_mask;
  bool is_param_mask_used;
  bool is_active;
  unsigned int id;
  unsigned int opt_id;
  double time;
  std::vector<int> proj_residuals;
  std::vector<int> inertial_residuals;
  std::vector<int> binary_residuals;
  std::vector<int> unary_residuals;
  std::vector<int> landmarks;
   aligned_vector<Sophus::SE3Group<Scalar> > t_sw;

  const Sophus::SE3Group<Scalar>& GetTsw(const unsigned int cam_id,
                                         const calibu::CameraRigT<Scalar>& rig,
                                         const bool use_internal_t_sw) {
    while (t_sw.size() <= cam_id) {
      if (use_internal_t_sw == false) {
        t_sw.push_back((t_wp * rig.cameras[t_sw.size()].T_wc).inverse());
      } else {
        // this needs to be modified to work with stereo
        t_sw.push_back((t_wp * t_vs).inverseb());
      }
    }
    return t_sw[cam_id];
  }
};

///
/// \brief GetGravityVector Returns the 3d gravity vector from the 2d
/// direction vector
/// \param direction The 2d gravity direction vector
/// \param g Gravity of 1 g in m/s^2
/// \return The 3d gravity vecto
///
template<typename Scalar>
static Eigen::Matrix<Scalar, 3, 1> GetGravityVector(
    const Eigen::Matrix<Scalar, 2, 1>& dir,
    Scalar g = static_cast<Scalar>(kGravity)) {
  Scalar sp = sin(dir[0]);
  Scalar cp = cos(dir[0]);
  Scalar sq = sin(dir[1]);
  Scalar cq = cos(dir[1]);
  Eigen::Matrix<Scalar, 3, 1> vec(cp * sq, -sp, cp * cq);
  vec *= -g;
  return vec;
}

template<typename Scalar = double>
struct ImuCalibrationT {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ImuCalibrationT(const Sophus::SE3Group<Scalar>& t_vs,
                  const Eigen::Matrix<Scalar, 3, 1>& b_g,
                  const Eigen::Matrix<Scalar, 3, 1>& b_a,
                  const Eigen::Matrix<Scalar, 2, 1>& g)
      : t_vs(t_vs),
        b_g(b_g),
        b_a(b_a),
        g(g),
        g_vec(GetGravityVector(g)),
        r((Eigen::Matrix<Scalar, 6, 1>() <<
           IMU_GYRO_UNCERTAINTY,
           IMU_GYRO_UNCERTAINTY,
           IMU_GYRO_UNCERTAINTY,
           IMU_ACCEL_UNCERTAINTY,
           IMU_ACCEL_UNCERTAINTY,
           IMU_ACCEL_UNCERTAINTY).finished().asDiagonal()) {
  }

  /// \brief Calibration from vehicle to sensor frame (monocular for now)
  Sophus::SE3Group<Scalar> t_vs;

  /// \brief Gyroscope bias vector
  Eigen::Matrix<Scalar, 3, 1> b_g;

  /// \brief Accelerometer bias vector
  Eigen::Matrix<Scalar, 3, 1> b_a;

  /// \brief Gravity vector (2D, parametrized by roll and pitch of the
  /// vector wrt the ground plane)
  Eigen::Matrix<Scalar, 2, 1> g;
  Eigen::Matrix<Scalar, 3, 1> g_vec;

  /// \brief Sensor uncertainty. The first 3 rows/cols are gyroscope
  /// and the last are accel
  Eigen::Matrix<Scalar, 6, 6> r;
};

template<typename Scalar>
///
/// \brief dGravity_dDirection Returns the jacobian associated with getting
/// the 3d gravity vector from the 2d direction
/// \param direction The 2d gravity direction vector
/// \param g Gravity of 1 g in m/s^2
/// \return The 3x2 jacobian matrix
///
static Eigen::Matrix<Scalar, 3, 2> dGravity_dDirection(
    const Eigen::Matrix<Scalar, 2, 1>& dir,
    Scalar g = visual_inertial_calibration::kGravity) {
  Scalar sp = sin(dir[0]);
  Scalar cp = cos(dir[0]);
  Scalar sq = sin(dir[1]);
  Scalar cq = cos(dir[1]);
  Eigen::Matrix<Scalar, 3, 2> vec;
  vec << -sp * sq, cp * cq, -cp, 0, -cq * sp, -cp * sq;
  vec *= -g;
  return vec;
}

template<typename Scalar = double>
struct ImuPoseT {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  explicit ImuPoseT(const PoseT<Scalar>& pose)
      : t_wp(pose.t_wp), v_w(pose.v_w),
        w_w(Eigen::Matrix<Scalar, 3, 1>::Zero()), time(pose.time) { }

  ImuPoseT(const Sophus::SE3Group<Scalar>& twp,
           const Eigen::Matrix<Scalar, 3, 1>& v,
           const Eigen::Matrix<Scalar, 3, 1>& w, double time)
      : t_wp(twp), v_w(v), w_w(w), time(time) { }

  operator Eigen::Matrix<Scalar, 10, 1>() {
    Eigen::Matrix<Scalar, 10, 1> res;
    res.template head<3>() = t_wp.translation();
    res.template segment<4>(3) = t_wp.unit_quaternion().coeffs();
    res.template tail<3>() = v_w;
    return res;
  }

  /// \brief pose in world coordinates
  Sophus::SE3Group<Scalar> t_wp;

  /// \brief velocity in world coordinates
  Eigen::Matrix<Scalar, 3, 1> v_w;

  /// \brief angular rates in world coordinates
  Eigen::Matrix<Scalar, 3, 1> w_w;

  /// \brief time in seconds
  double time;
};

template<typename Scalar = double>
struct ImuMeasurementT {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ImuMeasurementT() {}
  ImuMeasurementT(const Eigen::Matrix<Scalar, 3, 1>& w,
                  const Eigen::Matrix<Scalar, 3, 1>& a, double time)
      : w(w), a(a), time(time) { }

  /// \brief angular rates in inertial coordinates
  Eigen::Matrix<Scalar, 3, 1> w;
  /// \brief accelerations in inertial coordinates
  Eigen::Matrix<Scalar, 3, 1> a;
  double time;

  ImuMeasurementT operator*(const Scalar &rhs) {
    return ImuMeasurementT(w * rhs, a * rhs, time);
  }

  ImuMeasurementT operator+(const ImuMeasurementT &rhs) {
    return ImuMeasurementT(w + rhs.w, a + rhs.a, time);
  }
};

template<typename Scalar, int kParamSize>
struct ResidualT {
  unsigned int residual_id;
  unsigned int residual_offset;
  Scalar weight;
  Scalar orig_weight;
};

template<typename Scalar = double>
struct UnaryResidualT : public ResidualT<Scalar, 6> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static const unsigned int kResSize = 6;
  unsigned int pose_id;
  Sophus::SE3Group<Scalar> t_wp;
  Eigen::Matrix<Scalar, kResSize, 6> dz_dx;
  Eigen::Matrix<Scalar, 6, 1> residual;
};

template<typename Scalar = double>
struct BinaryResidualT : public ResidualT<Scalar, 6> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static const unsigned int kResSize = 6;
  unsigned int x1_id;
  unsigned int x2_id;
  Sophus::SE3Group<Scalar> t_ab;
  Eigen::Matrix<Scalar, kResSize, 6> dz_dx1;
  Eigen::Matrix<Scalar, kResSize, 6> dz_dx2;
  Eigen::Matrix<Scalar, 6, 1> residual;
};

template<typename Scalar = double, int LmSize = 1>
struct ProjectionResidualT : public ResidualT<Scalar, 6> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static const unsigned int kResSize = 2;
  Eigen::Matrix<Scalar, 2, 1> z;
  unsigned int x_meas_id;
  unsigned int x_ref_id;
  unsigned int landmark_id;
  unsigned int cam_id;

  Eigen::Matrix<Scalar, kResSize, LmSize> dz_dlm;
  Eigen::Matrix<Scalar, 2, 6> dz_dx_meas;
  Eigen::Matrix<Scalar, 2, 6> dz_dx_ref;
  Eigen::Matrix<Scalar, 2, Eigen::Dynamic> dz_dcam_params;
  Eigen::Matrix<Scalar, 2, 1> residual;
};

template<typename Scalar = double, int ResidualSize = 15, int PoseSize = 15>
struct ImuResidualT : public ResidualT<Scalar, PoseSize> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef ImuPoseT<Scalar> ImuPose;
  typedef ImuMeasurementT<Scalar> ImuMeasurement;
  static const unsigned int kResSize = ResidualSize;
  unsigned int pose1_id;
  unsigned int pose2_id;
  aligned_vector<ImuMeasurement> measurements;
  aligned_vector<ImuPose> poses;
  Eigen::Matrix<Scalar, kResSize, PoseSize> dz_dx1;
  Eigen::Matrix<Scalar, kResSize, PoseSize> dz_dx2;
  Eigen::Matrix<Scalar, kResSize, kResSize> cov_inv;
  Eigen::Matrix<Scalar, kResSize, 6> dz_dy;
  Eigen::Matrix<Scalar, 9, 2> dz_dg;
  Eigen::Matrix<Scalar, kResSize, 6> dz_db;
  Eigen::Matrix<Scalar, kResSize, 1> residual;

  static ImuPose IntegratePose(const ImuPose& pose,
                               const Eigen::Matrix<Scalar, 9, 1>& k,
                               Scalar dt,
                               Eigen::Matrix<Scalar, 10, 9>* pdy_dk = 0,
                               Eigen::Matrix<Scalar, 10, 10>* pdy_dy = 0) {
    const Sophus::SO3Group<Scalar> r_v2_v1(
        Sophus::SO3Group<Scalar>::exp(k.template segment<3>(3) * dt));

    ImuPose y = pose;
    // integrate translation
    y.t_wp.translation() += k.template head<3>() * dt;
    // integrate rotation using exp
    const Eigen::Quaternion<Scalar> q = (r_v2_v1.unit_quaternion()
                                         * pose.t_wp.so3().unit_quaternion());
    // unfortunately need to memcpy to avoid normalization
    memcpy(y.t_wp.so3().data(), q.coeffs().data(), sizeof(Scalar) * 4);
    // integrate velocity
    y.v_w += k.template tail<3>() * dt;

    // jacobian of output pose relative to the derivative
    if (pdy_dk != 0) {
      pdy_dk->setZero();
      // dt/dv
      pdy_dk->template block<3, 3>(0, 0) =
          Eigen::Matrix<Scalar, 3, 3>::Identity() * dt;
      // dq/dw
      pdy_dk->template block<4, 3>(3, 3) = dq1q2_dq1<Scalar>(
          pose.t_wp.so3().unit_quaternion())
          * dqExp_dw<Scalar>(k.template segment<3>(3) * dt) * dt;
      // dv/da
      pdy_dk->template block<3, 3>(7, 6) =
          Eigen::Matrix<Scalar, 3, 3>::Identity() * dt;
    }

    if (pdy_dy != 0) {
      pdy_dy->setZero();
      pdy_dy->template block<3, 3>(0, 0) =
          Eigen::Matrix<Scalar, 3, 3>::Identity();

      pdy_dy->template block<4, 4>(3, 3) = dq1q2_dq2(r_v2_v1.unit_quaternion());

      pdy_dy->template block<3, 3>(7, 7) =
          Eigen::Matrix<Scalar, 3, 3>::Identity();

      BA_TEST(_Test_IntegratePose_ExpJacobian(k, dt));
      BA_TEST(_Test_IntegratePose_StateKJacobian(pose, k, dt, *pdy_dk));
    }
    return y;
  }

  static Eigen::Matrix<Scalar, 9, 1> GetPoseDerivative(
      const ImuPose& pose, const Eigen::Matrix<Scalar, 3, 1>& g_w,
      const ImuMeasurement& z_start, const ImuMeasurement& z_end,
      const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 6, 1>& sf,
      Scalar dt,
      Eigen::Matrix<Scalar, 9, 6>* dk_db = 0,
      Eigen::Matrix<Scalar, 9, 10>* dk_dx = 0) {
    double alpha = (z_end.time - (z_start.time + dt))
        / (z_end.time - z_start.time);
    Eigen::Matrix<Scalar, 3, 1> zg = z_start.w * alpha
        + z_end.w * (1.0 - alpha);
    Eigen::Matrix<Scalar, 3, 1> za = z_start.a * alpha
        + z_end.a * (1.0 - alpha);

    // calculate derivatives at this point
    Eigen::Matrix<Scalar, 9, 1> deriv;
    // v (velocity)
    deriv.template head<3>() = pose.v_w;
    // w (angular rates)
    deriv.template segment<3>(3) = pose.t_wp.so3().Adj() *
        (zg.cwiseProduct(sf.template head<3>()) + bg);
    // a (acceleration)
    deriv.template segment<3>(6) = pose.t_wp.so3() *
        (za.cwiseProduct(sf.template tail<3>()) + ba) - g_w;

    if (dk_db != 0) {
      dk_db->setZero();
      dk_db->template block<3, 3>(3, 0) = pose.t_wp.so3().Adj();  // dw/dbg
      dk_db->template block<3, 3>(6, 3) = pose.t_wp.so3().matrix();  // da/dba
    }
    if (dk_dx != 0) {
      dk_dx->setZero();
      dk_dx->template block<3, 3>(0, 7) =
          Eigen::Matrix<Scalar, 3, 3>::Identity();  // dv/dv
      dk_dx->template block<3, 4>(3, 3) = dqx_dq(
          pose.t_wp.so3().unit_quaternion(), zg)
          + dqx_dq(pose.t_wp.so3().unit_quaternion(), bg);  // dw/dq
      dk_dx->template block<3, 4>(6, 3) = dqx_dq(
          pose.t_wp.so3().unit_quaternion(), za)
          + dqx_dq(pose.t_wp.so3().unit_quaternion(), ba);  // da/dq
    }
    return deriv;
  }

  static ImuPose IntegrateImu(const ImuPose& pose,
                              const ImuMeasurement& z_start,
                              const ImuMeasurement& z_end,
                              const Eigen::Matrix<Scalar, 3, 1>& bg,
                              const Eigen::Matrix<Scalar, 3, 1>& ba,
                              const Eigen::Matrix<Scalar, 6, 1>& sf,
                              const Eigen::Matrix<Scalar, 3, 1>& g,
                              Eigen::Matrix<Scalar, 10, 6>* dy_db_ptr = 0,
                              Eigen::Matrix<Scalar, 10, 10>* dy_dpose_ptr = 0,
                              Eigen::Matrix<Scalar, 10, 10>* c_prior = 0) {
    // construct the state matrix
    Scalar dt = z_end.time - z_start.time;
    if (dt == 0) {
      return pose;
    }

    ImuPose res = pose;
    Eigen::Matrix<Scalar, 9, 1> k;

    if (dy_db_ptr != 0 && dy_dpose_ptr != 0) {
      Eigen::Matrix<Scalar, 10, 6>& dy_db = *dy_db_ptr;
      Eigen::Matrix<Scalar, 10, 10>& dy_dy0 = *dy_dpose_ptr;

      Eigen::Matrix<Scalar, 9, 6> dk_db;
      Eigen::Matrix<Scalar, 9, 10> dk_dy;
      Eigen::Matrix<Scalar, 10, 9> dy_dk;
      Eigen::Matrix<Scalar, 10, 10> dy_dy;
      dy_db.setZero();
      dy_dy0.setIdentity();   // dy0_y0 starts at identity

      Eigen::Matrix<Scalar, 9, 9> c_k1, c_k2, c_k3, c_k4, c_k;
      Eigen::Matrix<Scalar, 10, 10> c_y1, c_y2, c_y3, c_res;

      const Eigen::Matrix<Scalar, 6, 1> cov_diag =
          (Eigen::Matrix<Scalar, 6, 1>() <<
           IMU_GYRO_UNCERTAINTY, IMU_GYRO_UNCERTAINTY,
           IMU_GYRO_UNCERTAINTY, IMU_ACCEL_UNCERTAINTY,
           IMU_ACCEL_UNCERTAINTY, IMU_ACCEL_UNCERTAINTY).finished();
      const Eigen::Matrix<Scalar, 6, 6> cov_meas = cov_diag.asDiagonal();

      const Eigen::Matrix<Scalar, 9, 1> k1 = GetPoseDerivative(
          pose, g, z_start, z_end, bg, ba, sf, 0, &dk_db, &dk_dy);

      // Calculate uncertainty of k1.
      if (c_prior != 0) {
        c_k1 = dk_dy * (*c_prior) * dk_dy.transpose() +
            dk_db * cov_meas * dk_db.transpose();
      }

      BA_TEST(_Test_IntegrateImu_KBiasJacobian(pose, z_start, z_end,
                                               bg, ba, g, dk_db));
      BA_TEST(_Test_IntegrateImu_KStateJacobian(pose, z_start, z_end, bg,
                                                ba, g, dk_dy));

      // total derivative of k1 wrt b: dk1/db = dG/db + dG/dy*dy/db
      const Eigen::Matrix<Scalar, 9, 6> dk1_db = dk_db + dk_dy * dy_db;
      const Eigen::Matrix<Scalar, 9, 10> dk1_dy = dk_dy * dy_dy0;
      const ImuPose y1 = IntegratePose(pose, k1, dt * 0.5, &dy_dk, &dy_dy);

      // dy1/db = dInt/db + dInt/dy*dy0/db + dInt/dk*dk/db
      // however dy0/db = 0 (only for y0), therefore we don't
      // need the second term, just dInt/dk and dInt/db.
      // but dInt/db is also 0, as the integration doesn't directly depend on b
      dy_db = dy_dk * dk1_db;
      dy_dy0 = dy_dy + dy_dk * dk1_dy;  // this is dy1_dy0

      // Calculate uncertainty of y1.
      if (c_prior != 0) {
        c_y1 = dy_dy * (*c_prior) * dy_dy.transpose() +
            dy_dk * c_k1 * dy_dk.transpose();
      }

      BA_TEST(_Test_IntegrateImu_StateStateJacobian(pose, k1, dy_dy, dt));

      const Eigen::Matrix<Scalar, 9, 1> k2 = GetPoseDerivative(
          y1, g, z_start, z_end, bg, ba, sf, dt / 2, &dk_db, &dk_dy);

      // Calculate uncertainty of k2.
      if (c_prior != 0) {
        c_k2 = dk_dy * c_y1 * dk_dy.transpose() +
            dk_db * cov_meas * dk_db.transpose();
      }

      const Eigen::Matrix<Scalar, 9, 6> dk2_db = dk_db + dk_dy * dy_db;
      const Eigen::Matrix<Scalar, 9, 10> dk2_dy = dk_dy * dy_dy0;
      const ImuPose y2 = IntegratePose(pose, k2, dt * 0.5, &dy_dk, &dy_dy);
      dy_db = dy_dk * dk2_db;
      dy_dy0 = dy_dy + dy_dk * dk2_dy;  // this is dy2_dy0

      // Calculate uncertainty of y2.
      if (c_prior != 0) {
        c_y2 = dy_dy * (*c_prior) * dy_dy.transpose() +
            dy_dk * c_k2 * dy_dk.transpose();
      }

      const Eigen::Matrix<Scalar, 9, 1> k3 = GetPoseDerivative(
          y2, g, z_start, z_end, bg, ba, sf, dt / 2, &dk_db, &dk_dy);

      // Calculate uncertainty of k3.
      if (c_prior != 0) {
        c_k3 = dk_dy * c_y2 * dk_dy.transpose() +
            dk_db * cov_meas * dk_db.transpose();
      }

      const Eigen::Matrix<Scalar, 9, 6> dk3_db = dk_db + dk_dy * dy_db;
      const Eigen::Matrix<Scalar, 9, 10> dk3_dy = dk_dy * dy_dy0;
      const ImuPose y3 = IntegratePose(pose, k3, dt, &dy_dk, &dy_dy);
      dy_db = dy_dk * dk3_db;
      dy_dy0 = dy_dy + dy_dk * dk3_dy;  // this is dy3_dy0

      // Calculate uncertainty of y3.
      if (c_prior != 0) {
        c_y3 = dy_dy * (*c_prior) * dy_dy.transpose() +
            dy_dk * c_k3 * dy_dk.transpose();
      }

      const Eigen::Matrix<Scalar, 9, 1> k4 = GetPoseDerivative(
          y3, g, z_start, z_end, bg, ba, sf, dt, &dk_db, &dk_dy);

      // Calculate uncertainty of k4.
      if (c_prior != 0) {
        c_k4 = dk_dy * c_y3 * dk_dy.transpose() +
            dk_db * cov_meas * dk_db.transpose();
      }

      const Eigen::Matrix<Scalar, 9, 6> dk4_db = dk_db + dk_dy * dy_db;
      const Eigen::Matrix<Scalar, 9, 10> dk4_dy = dk_dy * dy_dy0;

      k = (k1 + 2 * k2 + 2 * k3 + k4);

      // Calculate uncertainty of k4.
      if (c_prior != 0) {
        c_k = c_k1 + 2 * c_k2 + 2 * c_k3 + c_k4;
      }

      const Eigen::Matrix<Scalar, 9, 6> dk_total_db = dk1_db + 2 * dk2_db
          + 2 * dk3_db + dk4_db;
      const Eigen::Matrix<Scalar, 9, 10> dk_total_dy = dk1_dy + 2 * dk2_dy
          + 2 * dk3_dy + dk4_dy;

      res = IntegratePose(pose, k, dt / 6.0, &dy_dk, &dy_dy);
      dy_db = dy_dk * dk_total_db;
      dy_dy0 = dy_dy + dy_dk * dk_total_dy;

      // Calculate uncertainty of res.
      if (c_prior != 0) {
        c_res = dy_dk * c_k * dy_dk.transpose() +
            dy_dy * (*c_prior) * dy_dy.transpose();
        const Eigen::Matrix<Scalar, 10, 10> c_prop =
            dy_dy0 * (*c_prior) * dy_dy0.transpose();
        *c_prior = c_prop + dy_db * cov_meas * dy_db.transpose();
      }
    } else {
      const Eigen::Matrix<Scalar, 9, 1> k1 = GetPoseDerivative(
          pose, g, z_start, z_end, bg, ba, sf, 0);
      const ImuPose y1 = IntegratePose(pose, k1, dt * 0.5);
      const Eigen::Matrix<Scalar, 9, 1> k2 = GetPoseDerivative(
          y1, g, z_start, z_end, bg, ba, sf, dt / 2);
      const ImuPose y2 = IntegratePose(pose, k2, dt * 0.5);
      const Eigen::Matrix<Scalar, 9, 1> k3 = GetPoseDerivative(
          y2, g, z_start, z_end, bg, ba, sf, dt / 2);
      const ImuPose y3 = IntegratePose(pose, k3, dt);
      const Eigen::Matrix<Scalar, 9, 1> k4 = GetPoseDerivative(
          y3, g, z_start, z_end, bg, ba, sf, dt);

      k = (k1 + 2 * k2 + 2 * k3 + k4);
      res = IntegratePose(pose, k, dt / 6.0);
    }

    res.w_w = k.template segment<3>(3);
    res.time = z_end.time;
    return res;
  }

  static ImuPose IntegrateResidual(
      const PoseT<Scalar>& pose,
      aligned_vector<ImuMeasurement>& measurements,
      const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 3, 1>& g,
      aligned_vector<ImuPose>& poses_out,
      Eigen::Matrix<Scalar, 10, 6>* dpose_db = 0,
      Eigen::Matrix<Scalar, 10, 10>* dpose_dpose = 0,
      Eigen::Matrix<Scalar, 10, 10>* c_res = 0) {
    return IntegrateResidual(ImuPose(pose), measurements, bg, ba, g, poses_out,
                             dpose_db, dpose_dpose, c_res);
  }

  static ImuPose IntegrateResidual(
      ImuPose pose,
      const aligned_vector<ImuMeasurement>& measurements,
      const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 6, 1>& sf,
      const Eigen::Matrix<Scalar, 3, 1>& g,
      aligned_vector<ImuPose>& poses,
      Eigen::Matrix<Scalar, 10, 6>* dpose_db = 0,
      Eigen::Matrix<Scalar, 10, 10>* dpose_dpose = 0,
      Eigen::Matrix<Scalar, 10, 10>* c_res = 0) {
    const ImuPose orig_pose = pose;
    const ImuMeasurement* prev_meas = 0;
    poses.clear();
    poses.reserve(measurements.size() + 1);
    poses.push_back(pose);

    if (dpose_db != 0) {
      dpose_db->setZero();
    }

    if (dpose_dpose != 0) {
      dpose_dpose->setIdentity();
    }

    // integrate forward in time, and retain all the poses
    for (const ImuMeasurement& meas : measurements) {
      if (prev_meas != 0) {
        if (dpose_db != 0 || dpose_dpose != 0) {
          Eigen::Matrix<Scalar, 10, 6> dy_db;
          Eigen::Matrix<Scalar, 10, 10> dy_dy;
          const ImuPose y0 = pose;
          pose = IntegrateImu(pose, *prev_meas, meas, bg, ba, sf, g, &dy_db,
                              &dy_dy, c_res);

          BA_TEST(_Test_IntegrateImu_BiasJacobian(y0, *prev_meas, meas,
                                                  bg, ba, g, dy_db));
          BA_TEST(_Test_IntegrateImu_StateJacobian(y0, *prev_meas, meas, bg,
                                                   ba, g, dy_dy));

          // now push forward the jacobian. This calculates the total derivative
          // Jb = dG/dB + dG/dX * dX/dB where dG/dB is the jacobian of the
          // return values of IntegrateImu with respect to the bias values
          // (which is returned in dq_dBg and dv_dBa, as the other values are
          // 0). dG/dX is the jacobian of the IntegrateImu function with respect
          // to its inputs (a 10x10 matrix, but only dq2_dq1 is complex, and is
          // returned by IntegrateImu). dX/dB yis the jacobian from the previous
          // step, which is stored in Jb. The following is the addition and
          // multiplication unrolled into sparse operations.
          if (dpose_db != 0) {
            (*dpose_db) = dy_db + dy_dy * (*dpose_db);
          }

          if (dpose_dpose != 0) {
            *dpose_dpose = dy_dy * (*dpose_dpose);
          }
        } else {
          pose = IntegrateImu(pose, *prev_meas, meas, bg, ba, sf, g);
        }
        poses.push_back(pose);
      }
      prev_meas = &meas;
    }

    if (dpose_db != 0) {
      BA_TEST(_Test_IntegrateResidual_BiasJacobian(orig_pose, measurements, bg,
                                                   ba, g, *dpose_db));
    }

    if (dpose_dpose != 0) {
      BA_TEST(_Test_IntegrateResidual_StateJacobian(orig_pose, measurements,
                                                    bg, ba, g, *dpose_dpose));
    }

    return pose;
  }

  static bool _Test_IntegrateImu_BiasJacobian(
      const ImuPose& pose, const ImuMeasurement& prev_meas,
      const ImuMeasurement& meas, const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 3, 1>& g,
      const Eigen::Matrix<Scalar, 10, 6>& dpose_db) {
    Eigen::Matrix<Scalar, 10, 6> dpose_db_fd;
    const Scalar kEps = kTestingEps;
    for (int ii = 0; ii < 6; ++ii) {
      Eigen::Matrix<Scalar, 6, 1> bias;
      bias.template head<3>() = bg;
      bias.template tail<3>() = ba;

      Eigen::Matrix<Scalar, 6, 1> eps_vec = Eigen::Matrix<Scalar, 6, 1>::Zero();
      eps_vec[ii] = kEps;
      bias += eps_vec;
      ImuPose pose_plus = IntegrateImu(pose, prev_meas, meas,
                                       bias.template head<3>(),
                                       bias.template tail<3>(), g);

      Eigen::Matrix<Scalar, 10, 1> pose_vec_plus;
      pose_vec_plus.template head<3>() = pose_plus.t_wp.translation();
      pose_vec_plus.template segment<4>(3) = pose_plus.t_wp.so3()
          .unit_quaternion().coeffs();
      // do euler integration for now
      pose_vec_plus.template tail<3>() = pose_plus.v_w;

      bias.template head<3>() = bg;
      bias.template tail<3>() = ba;

      eps_vec[ii] = -kEps;
      bias += eps_vec;
      ImuPose pose_minus = IntegrateImu(pose, prev_meas, meas,
                                        bias.template head<3>(),
                                        bias.template tail<3>(), g);

      Eigen::Matrix<Scalar, 10, 1> pose_vec_minus;
      pose_vec_minus.template head<3>() = pose_minus.t_wp.translation();
      pose_vec_minus.template segment<4>(3) = pose_minus.t_wp.so3()
          .unit_quaternion().coeffs();
      // do euler integration for now
      pose_vec_minus.template tail<3>() = pose_minus.v_w;

      dpose_db_fd.col(ii) = (pose_vec_plus - pose_vec_minus) / (2 * kEps);
    }
    std::cout << "dpose_db = " << std::endl << dpose_db.format(kCleanFmt)
              << std::endl;
    std::cout << "dpose_db_fd = " << std::endl << dpose_db_fd.format(kCleanFmt)
              << std::endl;
    std::cout << "diff = " << std::endl
              << (dpose_db - dpose_db_fd).format(kCleanFmt) << " norm: "
              << (dpose_db - dpose_db_fd).norm() << std::endl;
    return (dpose_db - dpose_db_fd).norm() < kNormThreshold;
  }

  static bool _Test_IntegrateImu_StateJacobian(
      const ImuPose& pose, const ImuMeasurement& prev_meas,
      const ImuMeasurement& meas, const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 3, 1>& g,
      const Eigen::Matrix<Scalar, 10, 10>& dpose_dpose) {
    const Scalar kEps = kTestingEps;
    Eigen::Matrix<Scalar, 10, 10> dpose_dpose_df;
    for (int ii = 0; ii < 10; ++ii) {
      Eigen::Matrix<Scalar, 10, 1> eps_vec =
          Eigen::Matrix<Scalar, 10, 1>::Zero();
      eps_vec[ii] = kEps;
      ImuPose pos_eps = pose;
      pos_eps.t_wp.translation() += eps_vec.template head<3>();
      Eigen::Quaternion<Scalar> q_p = pos_eps.t_wp.so3().unit_quaternion();
      q_p.coeffs() += eps_vec.template segment<4>(3);
      memcpy(pos_eps.t_wp.so3().data(), q_p.coeffs().data(),
             sizeof(Scalar) * 4);
      pos_eps.v_w += eps_vec.template tail<3>();

      ImuPose pose_plus = IntegrateImu(pos_eps, prev_meas, meas, bg, ba, g);

      Eigen::Matrix<Scalar, 10, 1> pose_vec_plus;
      pose_vec_plus.template head<3>() = pose_plus.t_wp.translation();
      pose_vec_plus.template segment<4>(3) = pose_plus.t_wp.so3()
          .unit_quaternion().coeffs();
      // do euler integration for now
      pose_vec_plus.template tail<3>() = pose_plus.v_w;

      eps_vec[ii] = -kEps;
      pos_eps = pose;
      pos_eps.t_wp.translation() += eps_vec.template head<3>();
      Eigen::Quaternion<Scalar> q_m = pos_eps.t_wp.so3().unit_quaternion();
      q_m.coeffs() += eps_vec.template segment<4>(3);
      pos_eps.t_wp.so3() = Sophus::SO3Group<Scalar>(q_m);
      memcpy(pos_eps.t_wp.so3().data(), q_m.coeffs().data(),
             sizeof(Scalar) * 4);
      pos_eps.v_w += eps_vec.template tail<3>();

      ImuPose pose_minus = IntegrateImu(pos_eps, prev_meas, meas, bg, ba, g);

      Eigen::Matrix<Scalar, 10, 1> pose_vec_minus;
      pose_vec_minus.template head<3>() = pose_minus.t_wp.translation();
      pose_vec_minus.template segment<4>(3) = pose_minus.t_wp.so3()
          .unit_quaternion().coeffs();
      // do euler integration for now
      pose_vec_minus.template tail<3>() = pose_minus.v_w;

      dpose_dpose_df.col(ii) = (pose_vec_plus - pose_vec_minus) / (2 * kEps);
    }
    std::cout << "dpose_dpose= " << std::endl << dpose_dpose.format(kCleanFmt)
              << std::endl;
    std::cout << "dpose_dpose_df = " << std::endl
              << dpose_dpose_df.format(kCleanFmt) << std::endl;
    std::cout << "diff = " << std::endl
              << (dpose_dpose - dpose_dpose_df).format(kCleanFmt) << " norm: "
              << (dpose_dpose - dpose_dpose_df).norm() << std::endl;

    return (dpose_dpose - dpose_dpose_df).norm() < kNormThreshold;
  }

  static bool _Test_IntegrateResidual_BiasJacobian(
      const ImuPose& pose,
      aligned_vector<ImuMeasurement>& measurements,
      const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 3, 1>& g,
      const Eigen::Matrix<Scalar, 10, 6>& dpose_db) {
    const ImuMeasurement* prev_meas = 0;
    Eigen::Matrix<Scalar, 10, 6> dpose_db_fd;
    const Scalar kEps = kTestingEps;
    Eigen::Matrix<Scalar, 6, 1> bias_vec;
    bias_vec.template head<3>() = bg;
    bias_vec.template tail<3>() = ba;
    for (int ii = 0; ii < 6; ++ii) {
      Eigen::Matrix<Scalar, 6, 1> eps_vec = Eigen::Matrix<Scalar, 6, 1>::Zero();
      eps_vec[ii] = kEps;
      aligned_vector<ImuPose> poses;
      const Eigen::Matrix<Scalar, 6, 1> b_plus = bias_vec + eps_vec;
      ImuPose pose_plus = pose;
      prev_meas = 0;
      for (const ImuMeasurement& meas : measurements) {
        if (prev_meas != 0) {
          pose_plus = IntegrateImu(pose_plus, *prev_meas, meas,
                                   b_plus.template head<3>(),
                                   b_plus.template tail<3>(), g);
        }
        prev_meas = &meas;
      }
      Eigen::Matrix<Scalar, 10, 1> pose_vec_plus;
      pose_vec_plus.template head<3>() = pose_plus.t_wp.translation();
      pose_vec_plus.template segment<4>(3) = pose_plus.t_wp.so3()
          .unit_quaternion().coeffs();
      pose_vec_plus.template tail<3>() = pose_plus.v_w;

      eps_vec[ii] = -kEps;
      const Eigen::Matrix<Scalar, 6, 1> b_minus = bias_vec + eps_vec;
      poses.clear();
      ImuPose pose_minus = pose;
      prev_meas = 0;
      for (const ImuMeasurement& meas : measurements) {
        if (prev_meas != 0) {
          pose_minus = IntegrateImu(pose_minus, *prev_meas, meas,
                                    b_minus.template head<3>(),
                                    b_minus.template tail<3>(), g);
        }
        prev_meas = &meas;
      }
      Eigen::Matrix<Scalar, 10, 1> pos_minus_vec;
      pos_minus_vec.template head<3>() = pose_minus.t_wp.translation();
      pos_minus_vec.template segment<4>(3) = pose_minus.t_wp.so3()
          .unit_quaternion().coeffs();
      pos_minus_vec.template tail<3>() = pose_minus.v_w;

      dpose_db_fd.col(ii) = (pose_vec_plus - pos_minus_vec) / (2 * kEps);
    }
    std::cout << "dpose_db = " << std::endl << (dpose_db).format(kCleanFmt)
              << std::endl;
    std::cout << "dpose_db_fd = " << std::endl << dpose_db_fd.format(kCleanFmt)
              << std::endl;
    std::cout << "diff = " << std::endl
              << (dpose_db - dpose_db_fd).format(kCleanFmt) << " norm: "
              << (dpose_db - dpose_db_fd).norm() << std::endl;
    return (dpose_db - dpose_db_fd).norm() < kNormThreshold;
  }

  static bool _Test_IntegrateResidual_StateJacobian(
      const ImuPose& pose,
      const aligned_vector<ImuMeasurement>& measurements,
      const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 3, 1>& g,
      const Eigen::Matrix<Scalar, 10, 10>& dpose_dpose) {
    const ImuMeasurement* prev_meas = 0;
    Eigen::Matrix<Scalar, 10, 10> dpose_dpose_fd;
    const Scalar kEps = kTestingEps;
    for (int ii = 0; ii < 10; ++ii) {
      Eigen::Matrix<Scalar, 10, 1> eps_vec =
          Eigen::Matrix<Scalar, 10, 1>::Zero();
      eps_vec[ii] += kEps;
      ImuPose pose_plus = pose;
      pose_plus.t_wp.translation() += eps_vec.template head<3>();
      Eigen::Quaternion<Scalar> q_p = pose_plus.t_wp.so3().unit_quaternion();
      q_p.coeffs() += eps_vec.template segment<4>(3);
      memcpy(pose_plus.t_wp.so3().data(), q_p.coeffs().data(),
             sizeof(Scalar) * 4);
      pose_plus.v_w += eps_vec.template tail<3>();

      prev_meas = 0;
      for (const ImuMeasurement& meas : measurements) {
        if (prev_meas != 0) {
          pose_plus = IntegrateImu(pose_plus, *prev_meas, meas, bg, ba, g);
        }
        prev_meas = &meas;
      }

      Eigen::Matrix<Scalar, 10, 1> pose_vec_plus;
      pose_vec_plus.template head<3>() = pose_plus.t_wp.translation();
      pose_vec_plus.template segment<4>(3) = pose_plus.t_wp.so3()
          .unit_quaternion().coeffs();
      // do euler integration for now
      pose_vec_plus.template tail<3>() = pose_plus.v_w;

      eps_vec[ii] -= 2 * kEps;
      ImuPose pose_minus = pose;
      pose_minus.t_wp.translation() += eps_vec.template head<3>();
      Eigen::Quaternion<Scalar> q_m = pose_minus.t_wp.so3().unit_quaternion();
      q_m.coeffs() += eps_vec.template segment<4>(3);
      pose_minus.t_wp.so3() = Sophus::SO3Group<Scalar>(q_m);
      memcpy(pose_minus.t_wp.so3().data(), q_m.coeffs().data(),
             sizeof(Scalar) * 4);
      pose_minus.v_w += eps_vec.template tail<3>();

      prev_meas = 0;
      for (const ImuMeasurement& meas : measurements) {
        if (prev_meas != 0) {
          pose_minus = IntegrateImu(pose_minus, *prev_meas, meas, bg, ba, g);
        }
        prev_meas = &meas;
      }

      Eigen::Matrix<Scalar, 10, 1> pose_vec_minus;
      pose_vec_minus.template head<3>() = pose_minus.t_wp.translation();
      pose_vec_minus.template segment<4>(3) = pose_minus.t_wp.so3()
          .unit_quaternion().coeffs();
      // do euler integration for now
      pose_vec_minus.template tail<3>() = pose_minus.v_w;

      dpose_dpose_fd.col(ii) = (pose_vec_plus - pose_vec_minus) / (2 * kEps);
    }
    std::cout << "dpose_dpose = " << std::endl
              << (dpose_dpose).format(kCleanFmt) << std::endl;
    std::cout << "dpose_dpose_fd = " << std::endl
              << dpose_dpose_fd.format(kCleanFmt) << std::endl;
    std::cout << "diff = " << std::endl
              << (dpose_dpose - dpose_dpose_fd).format(kCleanFmt) << " norm: "
              << (dpose_dpose - dpose_dpose_fd).norm() << std::endl;

    return (dpose_dpose - dpose_dpose_fd).norm() < kNormThreshold;
  }

  static bool _Test_IntegrateImu_KBiasJacobian(
      const ImuPose& pose, const ImuMeasurement& z_start,
      const ImuMeasurement& z_end, const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 3, 1>& g,
      const Eigen::Matrix<Scalar, 9, 6>& dk_db) {
    const Scalar kEps = kTestingEps;
    Eigen::Matrix<Scalar, 9, 6> dk_db_fd;
    for (int ii = 0; ii < 6; ++ii) {
      Eigen::Matrix<Scalar, 6, 1> bias_vec;
      bias_vec.template head<3>() = bg;
      bias_vec.template tail<3>() = ba;

      Eigen::Matrix<Scalar, 6, 1> eps_vec = Eigen::Matrix<Scalar, 6, 1>::Zero();
      eps_vec[ii] += kEps;
      bias_vec += eps_vec;
      Eigen::Matrix<Scalar, 9, 1> k1_plus = GetPoseDerivative(
          pose, g, z_start, z_end, bias_vec.template head<3>(),
          bias_vec.template tail<3>(), 0);

      bias_vec.template head<3>() = bg;
      bias_vec.template tail<3>() = ba;

      eps_vec[ii] -= 2 * kEps;
      bias_vec += eps_vec;
      Eigen::Matrix<Scalar, 9, 1> k1_minus = GetPoseDerivative(
          pose, g, z_start, z_end, bias_vec.template head<3>(),
          bias_vec.template tail<3>(), 0);
      dk_db_fd.col(ii) = (k1_plus - k1_minus) / (2 * kEps);
    }
    std::cout << "dk_db = " << std::endl << dk_db.format(kCleanFmt)
              << std::endl;
    std::cout << "dk_db_fd = " << std::endl << dk_db_fd.format(kCleanFmt)
              << std::endl;
    std::cout << "dk_db-diff = " << std::endl
              << (dk_db - dk_db_fd).format(kCleanFmt) << "norm: "
              << (dk_db - dk_db_fd).norm() << std::endl;

    return (dk_db - dk_db_fd).norm() < kNormThreshold;
  }

  static bool _Test_IntegrateImu_KStateJacobian(
      const ImuPose& pose, const ImuMeasurement& z_start,
      const ImuMeasurement& z_end, const Eigen::Matrix<Scalar, 3, 1>& bg,
      const Eigen::Matrix<Scalar, 3, 1>& ba,
      const Eigen::Matrix<Scalar, 3, 1>& g,
      const Eigen::Matrix<Scalar, 9, 10>& dk_dy) {
    const Scalar kEps = kTestingEps;
    Eigen::Matrix<Scalar, 9, 10> dk_dy_fd;
    for (int ii = 0; ii < 10; ++ii) {
      Eigen::Matrix<Scalar, 10, 1> eps_vec =
          Eigen::Matrix<Scalar, 10, 1>::Zero();
      eps_vec[ii] += kEps;
      ImuPose pose_eps = pose;
      pose_eps.t_wp.translation() += eps_vec.template head<3>();
      Eigen::Quaternion<Scalar> q_p = pose_eps.t_wp.so3().unit_quaternion();
      q_p.coeffs() += eps_vec.template segment<4>(3);
      memcpy(pose_eps.t_wp.so3().data(), q_p.coeffs().data(),
             sizeof(Scalar) * 4);
      pose_eps.v_w += eps_vec.template tail<3>();
      Eigen::Matrix<Scalar, 9, 1> k1_plus = GetPoseDerivative(pose_eps, g,
                                                              z_start, z_end,
                                                              bg, ba, 0);

      eps_vec[ii] -= 2 * kEps;
      pose_eps = pose;
      pose_eps.t_wp.translation() += eps_vec.template head<3>();
      Eigen::Quaternion<Scalar> q_m = pose_eps.t_wp.so3().unit_quaternion();
      q_m.coeffs() += eps_vec.template segment<4>(3);
      pose_eps.t_wp.so3() = Sophus::SO3Group<Scalar>(q_m);
      memcpy(pose_eps.t_wp.so3().data(), q_m.coeffs().data(),
             sizeof(Scalar) * 4);
      pose_eps.v_w += eps_vec.template tail<3>();
      Eigen::Matrix<Scalar, 9, 1> k1_minus = GetPoseDerivative(pose_eps, g,
                                                               z_start, z_end,
                                                               bg, ba, 0);

      dk_dy_fd.col(ii) = (k1_plus - k1_minus) / (2 * kEps);
    }
    std::cout << "dk_dy= " << std::endl << dk_dy.format(kCleanFmt) << std::endl;
    std::cout << "dk_dy_fd = " << std::endl << dk_dy_fd.format(kCleanFmt)
              << std::endl;
    std::cout << "diff = " << std::endl << (dk_dy - dk_dy_fd).format(kCleanFmt)
              << "norm: " << (dk_dy - dk_dy_fd).norm() << std::endl;

    return (dk_dy - dk_dy_fd).norm() < kNormThreshold;
  }

  static bool _Test_IntegrateImu_StateStateJacobian(
      const ImuPose& pose, const Eigen::Matrix<Scalar, 9, 1>& k,
      const Eigen::Matrix<Scalar, 10, 10>& dy_dy, const Scalar dt) {
    const Scalar kEps = kTestingEps;
    Eigen::Matrix<Scalar, 10, 10> dy_dy_fd;
    for (int ii = 0; ii < 10; ++ii) {
      Eigen::Matrix<Scalar, 10, 1> eps_vec =
          Eigen::Matrix<Scalar, 10, 1>::Zero();
      eps_vec[ii] += kEps;
      ImuPose pose_eps = pose;
      pose_eps.t_wp.translation() += eps_vec.template head<3>();
      Eigen::Quaternion<Scalar> q_p = pose_eps.t_wp.so3().unit_quaternion();
      q_p.coeffs() += eps_vec.template segment<4>(3);
      memcpy(pose_eps.t_wp.so3().data(), q_p.coeffs().data(),
             sizeof(Scalar) * 4);
      pose_eps.v_w += eps_vec.template tail<3>();

      Eigen::Matrix<Scalar, 10, 1> pose_vec_plus;
      ImuPose posePlus = IntegratePose(pose_eps, k, dt * 0.5);
      pose_vec_plus.template head<3>() = posePlus.t_wp.translation();
      pose_vec_plus.template segment<4>(3) = posePlus.t_wp.so3()
          .unit_quaternion().coeffs();
      pose_vec_plus.template tail<3>() = posePlus.v_w;

      eps_vec[ii] -= 2 * kEps;
      pose_eps = pose;
      pose_eps.t_wp.translation() += eps_vec.template head<3>();
      Eigen::Quaternion<Scalar> q_m = pose_eps.t_wp.so3().unit_quaternion();
      q_m.coeffs() += eps_vec.template segment<4>(3);
      pose_eps.t_wp.so3() = Sophus::SO3Group<Scalar>(q_m);
      memcpy(pose_eps.t_wp.so3().data(), q_m.coeffs().data(),
             sizeof(Scalar) * 4);
      pose_eps.v_w += eps_vec.template tail<3>();

      Eigen::Matrix<Scalar, 10, 1> pose_vec_minus;
      ImuPose pose_minus = IntegratePose(pose_eps, k, dt * 0.5);
      pose_vec_minus.template head<3>() = pose_minus.t_wp.translation();
      pose_vec_minus.template segment<4>(3) = pose_minus.t_wp.so3()
          .unit_quaternion().coeffs();
      pose_vec_minus.template tail<3>() = pose_minus.v_w;

      dy_dy_fd.col(ii) = (pose_vec_plus - pose_vec_minus) / (2 * kEps);
    }
    std::cout << "dy_dy= " << std::endl << dy_dy.format(kCleanFmt) << std::endl;
    std::cout << "dy_dy_fd = " << std::endl << dy_dy_fd.format(kCleanFmt)
              << std::endl;
    std::cout << "diff = " << std::endl << (dy_dy - dy_dy_fd).format(kCleanFmt)
              << "norm: " << (dy_dy - dy_dy_fd).norm() << std::endl;

    return (dy_dy - dy_dy_fd).norm() < kNormThreshold;
  }

  static bool _Test_IntegratePose_ExpJacobian(
      const Eigen::Matrix<Scalar, 9, 1>& k, const Scalar dt) {
    const Scalar kEps = kTestingEps;
    Eigen::Matrix<Scalar, 4, 3> dexp_dw_fd;
    for (int ii = 0; ii < 3; ++ii) {
      Eigen::Matrix<Scalar, 3, 1> eps_vec = Eigen::Matrix<Scalar, 3, 1>::Zero();
      eps_vec[ii] += kEps;
      Eigen::Matrix<Scalar, 3, 1> k_plus = k.template segment<3>(3) * dt;
      k_plus += eps_vec;
      Eigen::Matrix<Scalar, 4, 1> res_Plus =
          Sophus::SO3Group<Scalar>::exp(k_plus).unit_quaternion().coeffs();

      eps_vec[ii] -= 2 * kEps;
      Eigen::Matrix<Scalar, 3, 1> k_minus = k.template segment<3>(3) * dt;
      k_minus += eps_vec;
      Eigen::Matrix<Scalar, 4, 1> res_Minus =
          Sophus::SO3Group<Scalar>::exp(k_minus).unit_quaternion().coeffs();

      dexp_dw_fd.col(ii) = (res_Plus - res_Minus) / (2 * kEps);
    }

    const Eigen::Matrix<Scalar, 3, 1> k_segment = k.template segment <3>(3);
    std::cout << "dexp_dw_fd= " << std::endl
              << dqExp_dw<Scalar>(k_segment * dt).format(kCleanFmt)
              << std::endl;
    std::cout << "dexp_dw_fd=" << std::endl << dexp_dw_fd.format(kCleanFmt)
              << std::endl;
    std::cout << "diff= " << std::endl
              << (dqExp_dw<Scalar>(k_segment * dt) -
                  dexp_dw_fd).format(kCleanFmt)
              << "norm: " << (dqExp_dw<Scalar>(k_segment * dt) -
                              dexp_dw_fd).norm()
              << std::endl;

    return (dqExp_dw<Scalar>(k.template segment<3>(3) * dt) - dexp_dw_fd)
        .norm() <
      kNormThreshold;
  }

  static bool _Test_IntegratePose_StateKJacobian(
      const ImuPose& pose, const Eigen::Matrix<Scalar, 9, 1>& k,
      const Scalar dt, const Eigen::Matrix<Scalar, 10, 9>& dpose_dk) {
    const Scalar kEps = kTestingEps;
    Eigen::Matrix<Scalar, 10, 9> dpose_dk_d;
    for (int ii = 0; ii < 9; ++ii) {
      Eigen::Matrix<Scalar, 9, 1> eps_vec = Eigen::Matrix<Scalar, 9, 1>::Zero();
      eps_vec[ii] += kEps;
      Eigen::Matrix<Scalar, 9, 1> k_plus = k;
      k_plus += eps_vec;
      Eigen::Matrix<Scalar, 10, 1> res_plus;
      res_plus.template head<3>() = pose.t_wp.translation()
          + k_plus.template head<3>() * dt;

      res_plus.template segment<4>(3) =
          (Sophus::SO3Group<Scalar>::exp(k_plus.template segment<3>(3) * dt).
           unit_quaternion() * pose.t_wp.so3().unit_quaternion()).coeffs();

      res_plus.template tail<3>() = pose.v_w + k_plus.template tail<3>() * dt;

      eps_vec[ii] -= 2 * kEps;
      Eigen::Matrix<Scalar, 9, 1> kMinus = k;
      kMinus += eps_vec;
      Eigen::Matrix<Scalar, 10, 1> res_Minus;
      res_Minus.template head<3>() = pose.t_wp.translation()
          + kMinus.template head<3>() * dt;
      res_Minus.template segment<4>(3) =
          (Sophus::SO3Group<Scalar>::exp(kMinus.template segment<3>(3) * dt) *
           pose.t_wp.so3()).unit_quaternion().coeffs();

      res_Minus.template tail<3>() = pose.v_w + kMinus.template tail<3>() * dt;

      dpose_dk_d.col(ii) = (res_plus - res_Minus) / (2 * kEps);
    }

    std::cout << "dpose_dk=" << std::endl << dpose_dk.format(kCleanFmt)
              << std::endl;
    std::cout << "dpose_dk_d=" << std::endl << dpose_dk_d.format(kCleanFmt)
              << std::endl;
    std::cout << "diff =" << std::endl
              << (dpose_dk - dpose_dk_d).format(kCleanFmt) << "norm: "
              << (dpose_dk - dpose_dk_d).norm() << std::endl;

    return (dpose_dk - dpose_dk_d).norm() < kNormThreshold;
  }
};
}  // namespace visual_inertial_calibration
#endif  // VISUAL_INERTIAL_CALIBRATION_TYPES_H_
