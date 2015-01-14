// This file is part of the BA Project.
//
// Copyright (C) 2013 George Washington University,
// Nima Keivan,
// Steven Lovegrove,
// Gabe Sibley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef VISUAL_INERTIAL_CALIBRATION_VICALIBRATOR_H_
#define VISUAL_INERTIAL_CALIBRATION_VICALIBRATOR_H_

// Compile against new ceres covarience interface.
// #define CALIBU_CERES_COVAR

// Compute calibration covariance (can run out of memory)
// #define COMPUTE_VICALIB_COVARIANCE

#include <memory>
#include <pthread.h>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <ceres/ceres.h>
#ifdef CALIBU_CERES_COVAR
#include <ceres/covariance.h>
#endif  // CALIBU_CERES_COVAR
#include <calibu/cam/CameraModel.h>
#include <calibu/cam/CameraXml.h>
#include <calibu/calib/CostFunctionAndParams.h>
#include <calibu/cam/CameraRig.h>
#include <calibu/calib/ReprojectionCostFunctor.h>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <visual-inertial-calibration/local-param-se3.h>
#include <visual-inertial-calibration/types.h>
#include <visual-inertial-calibration/interpolation-buffer.h>
#include <visual-inertial-calibration/ceres-cost-functions.h>

DECLARE_string(output_log_file);
DECLARE_bool(calibrate_imu);  // Defined in vicalib-engine.cc

namespace visual_inertial_calibration {

// Tie together a single camera and its position relative to the IMU.
struct CameraAndPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CameraAndPose(const calibu::CameraModel& camera, const Sophus::SE3d& T_ck)
      : camera(camera), T_ck(T_ck) {}

  calibu::CameraModel camera;
  Sophus::SE3d T_ck;
};

// Subclass to handle missing images from certain cameras.
template <typename Scalar>
struct VicalibFrame : public ImuPoseT<Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  explicit VicalibFrame(const PoseT<Scalar>& pose) : ImuPoseT<Scalar>(pose) {}

  VicalibFrame(const Sophus::SE3Group<Scalar>& twp,
               const Eigen::Matrix<Scalar, 3, 1>& v,
               const Eigen::Matrix<Scalar, 3, 1>& w, const double time)
      : ImuPoseT<Scalar>(twp, v, w, time) {}

  void SetHasMeasurementsFromCam(size_t cam_id, const bool val) {
    // We have to resize the has_measurements_from_cam array if
    // it's too small.
    while (has_measurements_from_cam.size() <= cam_id) {
      has_measurements_from_cam.push_back(false);
    }

    has_measurements_from_cam[cam_id] = val;
  }

  std::vector<bool> has_measurements_from_cam;
};

typedef SwitchedFullImuCostFunction<double> ViFullCost;

// Cost function for optimizing IMU and image calibrations.
class ImuCostFunctionAndParams : public calibu::CostFunctionAndParams {
 public:
  void set_index(const int idx) { index_ = idx; }
  void set_cost_functor(std::shared_ptr<ViFullCost> functor) {
    cost_functor_ = functor;
  }

  std::shared_ptr<ViFullCost> cost_functor() const { return cost_functor_; }
  int index() const { return index_; }

 protected:
  std::shared_ptr<ViFullCost> cost_functor_;
  int index_;
};

// Handles all the mathematics of calibration. Sets up Ceres problem
// and processes results.
class ViCalibrator : public ceres::IterationCallback {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Construct empty calibration object.
  ViCalibrator()
      : is_running_(false),
        fix_intrinsics_(false),
        loss_func_(new ceres::SoftLOneLoss(0.5), ceres::TAKE_OWNERSHIP),
        imu_buffer_(1000),
        imu_(Sophus::SE3d(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
             Eigen::Vector2d::Zero()),
        biases_(Vector6d::Zero()),
        scale_factors_(Vector6d::Ones()),
        imu_loss_func_(100),
        num_imu_residuals_(0),
        optimize_time_offset_(true) {
    prob_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options_.local_parameterization_ownership =
        ceres::DO_NOT_TAKE_OWNERSHIP;
    prob_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

    solver_options_.num_threads = 4;

#ifdef ANDROID
    solver_options_.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
#endif  // ANDROID

    solver_options_.update_state_every_iteration = true;
    solver_options_.function_tolerance = 1e-6;
    solver_options_.callbacks.push_back(this);
    solver_options_.trust_region_strategy_type = ceres::DOGLEG;
    problem_.reset(new ceres::Problem(prob_options_));

    Clear();
  }

  virtual ~ViCalibrator() {}

  // Return the root mean squared error of the camera reprojections.
  std::vector<double> GetCameraProjRMSE() const { return camera_proj_rmse_; }

  // Write XML file containing configuration of camera rig.
  void WriteCameraModels(const std::string& filename) {
    calibu::CameraRig rig;

    for (size_t c = 0; c < cameras_.size(); ++c) {
      // Rdfrobotics.inverse is multiplied so that T_ck does not bake
      // the robotics (imu) to vision coordinate transform d
      if (FLAGS_calibrate_imu) {
        cameras_[c]->camera.SetRDF(calibu::RdfRobotics.matrix());
        rig.Add(cameras_[c]->camera,
                cameras_[c]->T_ck.inverse() *
                    Sophus::SE3d(calibu::RdfRobotics.inverse(),
                                 Eigen::Vector3d::Zero()));
      } else {
        // The RDF must be set to identity (computer vision).
        cameras_[c]->camera.SetRDF(calibu::RdfVision.matrix());

        rig.Add(cameras_[c]->camera, cameras_[c]->T_ck.inverse());
      }
    }

    WriteXmlRig(filename, rig);
  }

  // Clear all cameras / constraints.
  void Clear() {
    Stop();
    t_wk_.clear();
    cameras_.clear();
    proj_costs_.clear();
    imu_costs_.clear();
    mse_ = 0;
    num_imu_residuals_ = 0;
    num_iterations_ = 0;
    is_bias_active_ = false;
    is_scale_factor_active_ = false;
    is_inertial_active_ = false;
    is_visual_active_ = true;
    optimize_rotation_only_ = true;
    is_finished_ = false;
    is_gravity_initialized_ = false;
  }

  // Externally adjust which parts of the optimization are active.
  void SetOptimizationFlags(bool bias_active, bool inertial_active,
                            bool rotation_only, bool optimize_imu_time_offset) {
    CHECK(!is_running_);
    is_scale_factor_active_ = bias_active;
    is_bias_active_ = bias_active;
    is_inertial_active_ = inertial_active;
    optimize_rotation_only_ = rotation_only;
    optimize_time_offset_ = optimize_imu_time_offset;
  }

  // Start optimization thread to modify intrinsic / extrinsic parameters.
  void Start() {
    pthread_mutex_init(&update_mutex_, NULL);
    pthread_attr_init(&thread_attr_);
    pthread_attr_setdetachstate(&thread_attr_, PTHREAD_CREATE_JOINABLE);

    if (!is_running_) {
      should_run_ = true;
      pthread_create(&thread_, &thread_attr_, &ViCalibrator::SolveThreadStatic,
                     this);
    } else {
      LOG(WARNING) << "Already Running." << std::endl;
    }
  }

  // Configure the optimization's exit condition.
  void SetFunctionTolerance(const double tolerance) {
    CHECK(!is_running_);
    solver_options_.function_tolerance = tolerance;
  }

  // Retrieve the number of already completed Ceres outer-iterations.
  unsigned int GetNumIterations() { return num_iterations_; }

  // Access the current state of the IMU biases.
  Vector6d GetBiases() { return biases_; }

  Vector6d GetBiases() const { return biases_; }

  void SetSigmas(double gyro_sigma, double accel_sigma) {
    CHECK(!is_running_);
    gyro_sigma_ = gyro_sigma;
    accel_sigma_ = accel_sigma;
  }

  void SetTimeOffset(double offset)
  {
    imu_.time_offset_ = offset;
  }

  void SetBiases(const Vector6d& biases) {
    CHECK(!is_running_);
    biases_ = biases;
  }

  Vector6d GetScaleFactor() { return scale_factors_; }

  void SetScaleFactor(const Vector6d& scale_factors) {
    CHECK(!is_running_);
    scale_factors_ = scale_factors;
  }

  // Is the optimization currently active?
  bool IsRunning() { return is_running_ && !is_finished_; }

  // Stop optimization thread.
  void Stop() {
    if (is_running_) {
      should_run_ = false;
      try {
        pthread_join(thread_, NULL);
      }
      catch (std::system_error) {
        // thread already died.
        LOG(ERROR) << "Calibrator thread died prematurely.";
      }
    }
  }

  // Add camera to sensor rig. The returned ID should be used when adding
  // measurements for this camera.
  int AddCamera(const calibu::CameraModel& cam,
                const Sophus::SE3d& t_ck = Sophus::SE3d()) {
    CHECK(!is_running_);
    int id = cameras_.size();
    cameras_.push_back(
        std::unique_ptr<CameraAndPose>(new CameraAndPose(cam, t_ck)));
    cameras_.back()->camera.SetIndex(id);
    proj_costs_.resize(cameras_.size());
    camera_proj_rmse_.resize(cameras_.size());
    return id;
  }

  // Set whether intrinsics should be 'fixed' and left unchanged by the
  // minimization.
  void FixCameraIntrinsics(bool should_fix = true) {
    CHECK(!is_running_);
    fix_intrinsics_ = should_fix;
  }

  // Add frame to optimiser. The returned ID should be used when adding
  // target measurements for a given moment in time. Measurements given
  // for any camera for a given frame are assumed to be simultaneous, with
  // camera extrinsics equal between all cameras for each frame.
  int AddFrame(const Sophus::SE3d& t_wk, double time) {
    CHECK(!is_running_);
    pthread_mutex_lock(&update_mutex_);
    int id = t_wk_.size();
    VicalibFrame<double> pose(t_wk, Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero(), time);

    t_wk_.push_back(
        std::shared_ptr<VicalibFrame<double> >(new VicalibFrame<double>(pose)));
    pthread_mutex_unlock(&update_mutex_);

    return id;
  }

  // Add imu measurements.
  bool AddImuMeasurements(const Eigen::Vector3d& gyro,
                          const Eigen::Vector3d& accel, double time) {
    CHECK(!is_running_);
    if (time > imu_buffer_.end_time_) {
      imu_buffer_.AddElement(ImuMeasurementT<double>(gyro, accel, time));
      return true;
    } else {
      return false;
    }
  }

  // Add observation p_c of 3D feature P_w from 'camera' for 'frame'
  // 'camera' and 'frame' id's can be obtained by calls to AddCamera and
  // AddFrame respectively.
  void AddObservation(size_t frame, size_t camera_id,
                      const Eigen::Vector3d& p_w, const Eigen::Vector2d& p_c,
                      double time) {
    CHECK(!is_running_);
    pthread_mutex_lock(&update_mutex_);

    // Ensure index is valid
    while (NumFrames() < frame) {
      AddFrame(Sophus::SE3d(), time);
    }

    CHECK_LT(camera_id, NumCameras());

    // new camera pose to bundle adjust
    CameraAndPose& cp = *cameras_[camera_id];
    Sophus::SE3d& t_wk = t_wk_[frame]->t_wp_;

    // Indicate that we have measurements from this camera at this pose.
    t_wk_[frame]->SetHasMeasurementsFromCam(camera_id, true);

    // Create cost function
    calibu::CostFunctionAndParams* cost = new calibu::CostFunctionAndParams();

    calibu::CameraModelInterface& interface =
        cp.camera.GetCameraModelInterface();

    // Allocate and assign the correct cost function. Lifetimes are
    // handled by Calibu.
    if (dynamic_cast<calibu::CameraModelT<calibu::Fov>*>(
            &interface)) {  // NOLINT
      cost->Cost() = new ceres::AutoDiffCostFunction<
          ImuReprojectionCostFunctor<calibu::Fov>, 2,
          Sophus::SE3d::num_parameters, Sophus::SO3d::num_parameters, 3,
          calibu::Fov::NUM_PARAMS>(
          new ImuReprojectionCostFunctor<calibu::Fov>(p_w, p_c));

    } else if (dynamic_cast<calibu::CameraModelT<calibu::Poly2>*>(
                   &interface)) {  // NOLINT
      cost->Cost() = new ceres::AutoDiffCostFunction<
          ImuReprojectionCostFunctor<calibu::Poly2>, 2,
          Sophus::SE3d::num_parameters, Sophus::SO3d::num_parameters, 3,
          calibu::Poly2::NUM_PARAMS>(
          new ImuReprojectionCostFunctor<calibu::Poly2>(p_w, p_c));

    } else if (dynamic_cast<calibu::CameraModelT<calibu::Poly3>*>(
                   &interface)) {  // NOLINT
      cost->Cost() = new ceres::AutoDiffCostFunction<
          ImuReprojectionCostFunctor<calibu::Poly3>, 2,
          Sophus::SE3d::num_parameters, Sophus::SO3d::num_parameters, 3,
          calibu::Poly3::NUM_PARAMS>(
          new ImuReprojectionCostFunctor<calibu::Poly3>(p_w, p_c));

    } else if (dynamic_cast<calibu::CameraModelT<  // NOLINT
                   calibu::ProjectionKannalaBrandt>*>(&interface)) {
      cost->Cost() = new ceres::AutoDiffCostFunction<
          ImuReprojectionCostFunctor<calibu::ProjectionKannalaBrandt>, 2,
          Sophus::SE3d::num_parameters, Sophus::SO3d::num_parameters, 3,
          calibu::ProjectionKannalaBrandt::NUM_PARAMS>(
          new ImuReprojectionCostFunctor<calibu::ProjectionKannalaBrandt>(p_w,
                                                                          p_c));

    } else {
      LOG(FATAL) << "Don't know how to optimize CameraModel: "
                 << interface.Type();
    }

    cost->Params() = {t_wk.data(),                  cp.T_ck.so3().data(),
                      cp.T_ck.translation().data(), cp.camera.data()};

    cost->Loss() = &loss_func_;
    proj_costs_[camera_id]
        .push_back(std::unique_ptr<calibu::CostFunctionAndParams>(cost));

    pthread_mutex_unlock(&update_mutex_);
  }

  // Return number of synchronised camera rig frames.
  size_t NumFrames() const { return t_wk_.size(); }

  // Return pose of camera rig frame i.
  std::shared_ptr<VicalibFrame<double> > GetFrame(size_t i) {
    CHECK_LT(i, t_wk_.size()) << "GetFrame(): Frame index is greater than "
                              << "number of frames we have";
    return t_wk_[i];
  }

  // Return number of cameras in camera rig.
  size_t NumCameras() const { return cameras_.size(); }

  // Return the buffer storing IMU measurements.
  const InterpolationBufferT<ImuMeasurementT, double>& imu_buffer() {
    return imu_buffer_;
  }

  // Return camera i of camera rig.
  CameraAndPose& GetCamera(size_t i) {
    CHECK_LT(i, cameras_.size()) << "GetCamera(): Camera index is greater than "
                                 << "number of cameras we have";
    return *cameras_[i];
  }

  CameraAndPose GetCamera(size_t i) const {
    CHECK_LT(i, cameras_.size()) << "GetCamera(): Camera index is greater than "
                                 << "number of cameras we have";
    return *cameras_[i];
  }

  // Return current Mean Square reprojection Error - the objective function
  // being minimised by optimization.
  double MeanSquaredError() const { return mse_; }

  const aligned_vector<ImuPoseT<double> > GetIntegrationPoses(unsigned int id) {
    aligned_vector<ImuPoseT<double> > poses;
    if (is_inertial_active_ && !optimize_rotation_only_) {
      if (id <= t_wk_.size() - 1) {
        const visual_inertial_calibration::ImuPoseT<double>& prev_pose =
            *(t_wk_[id]);
        const visual_inertial_calibration::ImuPoseT<double>& pose =
            *(t_wk_[id + 1]);

        // get all the imu measurements between the two poses
        aligned_vector<ImuMeasurementT<double> > measurements;
        imu_buffer_.GetRange(prev_pose.time_, pose.time_, imu_.time_offset_,
                             &measurements);

        if (!measurements.empty()) {
          visual_inertial_calibration::ImuResidualT<double>::IntegrateResidual(
              prev_pose, measurements, biases_.head<3>(), biases_.tail<3>(),
              scale_factors_,
              visual_inertial_calibration::GetGravityVector<double>(
                  imu_.g_, visual_inertial_calibration::gravity()),
              poses);
        }
      }
    }
    return poses;
  }

  // Print summary of calibration.
  void PrintResults() {
    LOG(INFO) << "------------------------------------------" << std::endl;
    for (size_t c = 0; c < cameras_.size(); ++c) {
      LOG(INFO) << "Camera: " << c << std::endl;
      LOG(INFO) << cameras_[c]->camera.GenericParams().transpose() << std::endl;
      LOG(INFO) << cameras_[c]->T_ck.matrix();
      LOG(INFO) << std::endl;
    }
  }

 protected:
  // Build the optimization problem in Ceres.
  void SetupProblem(const std::shared_ptr<ceres::Problem>& problem) {
    covariance_params_.clear();
    covariance_names_.clear();
    projection_residuals_.clear();
    projection_residuals_.resize(cameras_.size());

    pthread_mutex_lock(&update_mutex_);

    // Add parameters
    std::stringstream css;
    for (size_t c = 0; c < cameras_.size(); ++c) {
      problem->AddParameterBlock(cameras_[c]->T_ck.so3().data(), 4,
                                 &local_param_so3_);
      covariance_params_.push_back(cameras_[c]->T_ck.so3().data());
      css.str("");
      css << "c[" << c << "].q_ck:(4)";
      covariance_names_.push_back(css.str());

      problem->AddParameterBlock(cameras_[c]->T_ck.translation().data(), 3);
      covariance_params_.push_back(cameras_[c]->T_ck.translation().data());
      css.str("");
      css << "c[" << c << "].p_ck:(3)";
      covariance_names_.push_back(css.str());

      if (c == 0) {
        if (!is_inertial_active_) {
          problem->SetParameterBlockConstant(cameras_[c]->T_ck.so3().data());
          problem->SetParameterBlockConstant(
              cameras_[c]->T_ck.translation().data());
        } else {
          problem->SetParameterBlockVariable(cameras_[c]->T_ck.so3().data());
          if (optimize_rotation_only_) {
            problem->SetParameterBlockConstant(
                cameras_[c]->T_ck.translation().data());
          } else {
            problem->SetParameterBlockVariable(
                cameras_[c]->T_ck.translation().data());
          }
        }
      }

      problem->AddParameterBlock(cameras_[c]->camera.data(),
                                 cameras_[c]->camera.NumParams());
      if (fix_intrinsics_) {
        problem->SetParameterBlockConstant(cameras_[c]->camera.data());
      } else {
        covariance_params_.push_back(cameras_[c]->camera.data());

        css.str("");
        css << "c[" << c << "].params:(" << cameras_[c]->camera.NumParams()
            << ")";
        covariance_names_.push_back(css.str());
      }
    }

    for (size_t jj = 0; jj < t_wk_.size(); ++jj) {
      problem->AddParameterBlock(t_wk_[jj]->t_wp_.data(), 7, &local_param_se3_);

      // add an imu cost residual if we have not yet for this frame
      if (jj >= num_imu_residuals_) {
        // get all the imu measurements between these two poses, and add
        // them to a vector
        if (jj > 0) {
          ImuCostFunctionAndParams* cost = new ImuCostFunctionAndParams();

          // check if there are IMU measurements between two frames.
          std::shared_ptr<ViFullCost> cost_functor(new ViFullCost(
              &imu_buffer_, t_wk_[jj - 1]->time_, t_wk_[jj]->time_,
              Eigen::Matrix<double, 9, 9>::Identity() * 500,
              &optimize_rotation_only_));


          cost->Cost() = new ceres::AutoDiffCostFunction<
              ViFullCost, 9, 7, 7, 3, 3, 2, 6, 6, 1>(cost_functor.get());

          cost->Loss() = &imu_loss_func_;

          cost->set_index(jj - 1);
          cost->set_cost_functor(cost_functor);

          cost->Params() = std::vector<double*>{
              t_wk_[jj]->t_wp_.data(), t_wk_[jj - 1]->t_wp_.data(),
              t_wk_[jj]->v_w_.data(),  t_wk_[jj - 1]->v_w_.data(),
              imu_.g_.data(),          biases_.data(),
              scale_factors_.data(),   &imu_.time_offset_};

          imu_costs_.push_back(std::unique_ptr<ImuCostFunctionAndParams>(cost));
        }
        ++num_imu_residuals_;
      }
    }

    // Add costs
    if (is_visual_active_) {
      for (size_t jj = 0; jj < cameras_.size(); ++jj) {
        for (size_t kk = 0; kk < proj_costs_[jj].size(); ++kk) {
          calibu::CostFunctionAndParams& cost = *(proj_costs_[jj][kk]);
          projection_residuals_[jj].push_back(problem->AddResidualBlock(
              cost.Cost(), cost.Loss(), cost.Params()));
        }
      }
    }

    if (FLAGS_calibrate_imu && is_inertial_active_) {
      for (size_t kk = 0; kk < imu_costs_.size(); ++kk) {
        calibu::CostFunctionAndParams& cost = *imu_costs_[kk];
        problem->AddResidualBlock(cost.Cost(), cost.Loss(), cost.Params());
      }

      // only do this once
      if (!is_bias_active_) {
        LOG(INFO) << "Setting bias terms to constant... ";
        problem->SetParameterBlockConstant(biases_.data());
      }

      if (!is_scale_factor_active_) {
        LOG(INFO) << "Setting scale factor terms to constant... ";
        problem->SetParameterBlockConstant(scale_factors_.data());
      }

      if (!optimize_time_offset_) {
        LOG(INFO) << "Setting time offset term to constant...";
        problem->SetParameterBlockConstant(&imu_.time_offset_);
      }
    }
    pthread_mutex_unlock(&update_mutex_);
  }

  // Entry point for background solver thread.
  static void* SolveThreadStatic(void* void_vicalibrator) {
    CHECK(void_vicalibrator);
    ViCalibrator* vicalibrator = static_cast<ViCalibrator*>(void_vicalibrator);
    vicalibrator->SolveThread();
    return NULL;
  }

  // Ceres callback after every iteration.
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    UpdateImuWeights();

    ++num_iterations_;

    // Update the mse at the end of the iteration.
    mse_ = summary.cost / problem_->NumResiduals();

    const char* kReportRowFormat =
        "% 4d: f:% 8e d:% 3.2e g:% 3.2e h:% 3.2e "
        "rho:% 3.2e mu:% 3.2e eta:% 3.2e li:% 3d";
    char buffer[5000];
    snprintf(buffer, sizeof(buffer), kReportRowFormat, summary.iteration,
             summary.cost, summary.cost_change, summary.gradient_max_norm,
             summary.step_norm, summary.relative_decrease,
             summary.trust_region_radius, summary.eta,
             summary.linear_solver_iterations);
    LOG(INFO) << buffer;

    // Terminate the optimization if the norm of the gradient is too small.
    LOG(INFO) << "Gradient norm is " << summary.gradient_norm;

    // Ceres calls this twice per iteration and gradient_norm is 0 (erroneously)
    static const double kNormThreshold = 1e-9;
    static const int kMaxSolverIterations = 200;
    if (summary.gradient_norm > 0 && summary.gradient_norm < kNormThreshold) {
      LOG(INFO) << "Terminating optimization as gradient norm = "
                << summary.gradient_norm << " < " << kNormThreshold;
      return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    } else if (summary.iteration > kMaxSolverIterations) {
      return ceres::SOLVER_ABORT;
    } else {
      return ceres::SOLVER_CONTINUE;
    }
  }

  void UpdateImuWeights() {
    // Go through all the cost functions and update the weights.
    if (is_inertial_active_ && !optimize_rotation_only_) {
      for (size_t ii = 0; ii < imu_costs_.size(); ++ii) {
        std::unique_ptr<ImuCostFunctionAndParams>& cost = imu_costs_[ii];
        aligned_vector<ImuMeasurementT<double> > measurements;
        cost->cost_functor()->GetMeasurements(imu_.time_offset_, &measurements);

        if (measurements.size() == 0) {
          continue;
        }

        // To propagate covariances, cast to double in order to stop
        // auto-diff on the covariance matrix.
        PoseT<double> start_pose;
        Sophus::SE3d t_2w = t_wk_[cost->index() + 1]->t_wp_.inverse();
        start_pose.t_wp_ = t_wk_[cost->index()]->t_wp_;
        start_pose.v_w_ = t_wk_[cost->index()]->v_w_;
        start_pose.time_ = measurements.front().time;
        aligned_vector<ImuPoseT<double> > poses_d;
        Eigen::Matrix<double, 10, 6> jb_q;
        Eigen::Matrix<double, 10, 10> c_imu_pose;
        c_imu_pose.setZero();

        const Eigen::Matrix<double, 6, 6> r(
            (Eigen::Matrix<double, 6, 1>() << powi(gyro_sigma_, 2),
             powi(gyro_sigma_, 2), powi(gyro_sigma_, 2),
             powi(accel_sigma_, 2), powi(accel_sigma_, 2),
             powi(accel_sigma_, 2))
                .finished()
                .asDiagonal());

        ImuPoseT<double> imu_pose = ImuResidualT<double>::IntegrateResidual(
            ImuPoseT<double>(start_pose), measurements, biases_.head<3>(),
            biases_.tail<3>(), scale_factors_,
            GetGravityVector(imu_.g_, gravity()), poses_d, &jb_q, nullptr,
            &c_imu_pose, &r);

        // calculate the derivative of the lie log with
        // respect to the tangent plane at Twa
        const Eigen::Matrix<double, 6, 7> dlog_dse3 =
            dLog_dSE3(imu_pose.t_wp_ * t_2w);

        // This is the 7x7 jacobian of the quaternion/translation
        // multiplication of
        // two transformations, with respect to the second
        // transformation (as the
        // operation is not commutative.)
        // For this derivation refer to page 22/23 of notes.
        const Eigen::Matrix<double, 7, 7> dt1t2_dt2 =
            dt1t2_dt1(imu_pose.t_wp_, t_2w);
        const Eigen::Matrix<double, 6, 7> dse3t1t2_dt2 = dlog_dse3 * dt1t2_dt2;

        // Transform the covariance through the multiplication
        // by t_2w as well as the SE3 log
        Eigen::Matrix<double, 9, 10> dse3t1t2v_dt2;
        dse3t1t2v_dt2.setZero();
        dse3t1t2v_dt2.topLeftCorner<6, 7>() = dse3t1t2_dt2;
        dse3t1t2v_dt2.bottomRightCorner<3, 3>().setIdentity();

        const Eigen::Matrix<double, 9, 9> cov =
            (dse3t1t2v_dt2 * c_imu_pose * dse3t1t2v_dt2.transpose()).inverse();

        // Get the residuals so that we can get the Mahalanobis distance.
        Eigen::Matrix<double, 9, 1> residuals;
        residuals.head<6>() = Sophus::SE3d::log(imu_pose.t_wp_ * t_2w);
        residuals.tail<3>() = imu_pose.v_w_ - t_wk_[cost->index() + 1]->v_w_;

        // Get the Mahalanobis distance given this residual.
        const double dist = residuals.transpose() * cov * residuals;
        LOG(INFO) << "Mahalanobis distance for res: " << ii << " is " << dist
                  << ", chi2inv(0.95, 9) = 16.9190";

        cost->cost_functor()->weight_sqrt_ = cov.sqrt();
      }
    }
  }

  // Calculate the covariance of the optimization.
  Eigen::MatrixXd GetSolutionCovariance(
      const std::shared_ptr<ceres::Problem>& problem) {
    LOG(INFO) << "Computing solution covariance." << std::endl;
    ceres::Covariance::Options op;
    ceres::Covariance covariance(op);
    std::vector<std::pair<const double*, const double*> > covariance_blocks;

    // Populate the covariance blocks.
    int num_params = 0;
    for (double* x : covariance_params_) {
      for (double* y : covariance_params_) {
        covariance_blocks.push_back(std::make_pair(x, y));
      }
      num_params += problem->ParameterBlockSize(x);
    }

    Eigen::MatrixXd covariance_mat(num_params, num_params);

    // Compute the covariance.
    if (covariance.Compute(covariance_blocks, problem.get())) {
      // Now readout the covariance into a giant matrix.
      int id_x = 0;
      for (double* x : covariance_params_) {
        int size_x = problem->ParameterBlockSize(x);
        int id_y = 0;
        for (double* y : covariance_params_) {
          int size_y = problem->ParameterBlockSize(y);

          // Create a row-major eigen matrix to take the result.
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
              block(size_x, size_y);
          covariance.GetCovarianceBlock(x, y, block.data());
          covariance_mat.block(id_x, id_y, size_x, size_y) = block;
          id_y += size_y;
        }
        id_x += size_x;
      }
      LOG(INFO) << "Covariance calculated for " << covariance_params_.size()
                << " blocks: " << std::endl;

      // Print the covariance column names.
      std::stringstream ss;
      for (const std::string& string : covariance_names_) {
        ss << string << " ";
      }

      LOG(INFO) << ss.str();

      LOG(INFO) << "Solution covariance: " << std::endl << covariance_mat
                << std::endl;
    } else {
      LOG(INFO) << "Failed to compute covariance...";
    }

    return covariance_mat;
  }

  // Runs the optimization in a background thread.
  void SolveThread() {
    is_running_ = true;

    while (should_run_ && !is_finished_) {
      SetupProblem(problem_);

      // Attempt to estimate the gravity vector if inertial constraints
      // are active.
      if (is_inertial_active_ && !optimize_rotation_only_ &&
          !is_gravity_initialized_) {
        int idx = t_wk_.size() / 2;
        std::shared_ptr<VicalibFrame<double> > frame = t_wk_[idx];
        ImuMeasurementT<double> meas = imu_buffer_.GetElement(frame->time_);
        const Eigen::Vector3d g_b = meas.a_.normalized();
        // Rotate gravity into the world frame.
        const Eigen::Vector3d g_w = frame->t_wp_.so3() * g_b;
        // Calculate the roll/pitch angles.
        LOG(INFO) << "Body accel: " << g_b.transpose()
                  << " world accel: " << g_w.transpose();

        double p, q;
        p = asin(g_w[1]);
        q = asin(-g_w[0] / cos(p));
        imu_.g_ << p, q;

        LOG(INFO) << "Estimated gravity as " << imu_.g_.transpose();

        LOG(INFO) << "Gravity vector using estimated gravity is: "
                  << GetGravityVector(imu_.g_, gravity()).transpose();
        is_gravity_initialized_ = true;
      }

      // Crank optimization
      while (problem_->NumResiduals() > 0 && should_run_ && !is_finished_) {
        try {
          ceres::Solver::Summary summary;
          UpdateImuWeights();
          ceres::Solve(solver_options_, problem_.get(), &summary);

          // Evaluate the reprojection error for each camera.
          for (size_t ii = 0; ii < cameras_.size(); ++ii) {
            ceres::Problem::EvaluateOptions options;
            options.residual_blocks = projection_residuals_[ii];
            options.apply_loss_function = false;
            double cost;
            problem_->Evaluate(options, &cost, nullptr, nullptr, nullptr);
            camera_proj_rmse_[ii] =
                sqrt(cost / (projection_residuals_[ii].size()));

            LOG(INFO) << "Reprojection error for camera " << ii << ": "
                      << sqrt(cost) << " rmse: " << camera_proj_rmse_[ii]
                      << std::endl;
          }

          LOG(INFO) << summary.FullReport() << std::endl;

          mse_ = summary.final_cost / summary.num_residuals;
          if (summary.termination_type != ceres::NO_CONVERGENCE &&
              FLAGS_calibrate_imu) {
            if (!is_inertial_active_) {
              is_inertial_active_ = true;
              LOG(INFO) << "Activating inertial terms. Optimizing rotation "
                           "component of T_ck..." << std::endl;
            } else if (optimize_rotation_only_) {
              LOG(INFO) << "Finished optimizing rotations. Activating T_ck "
                           "translation optimization..." << std::endl;
              optimize_rotation_only_ = false;
            // } else if (!is_bias_active_) {
              is_bias_active_ = true;
              LOG(INFO) << "Activating bias terms... " << std::endl;
              problem_->SetParameterBlockVariable(biases_.data());
            } else if (!is_scale_factor_active_) {
              is_scale_factor_active_ = true;
              LOG(INFO) << "Activating scale factor terms... " << std::endl;
              problem_->SetParameterBlockVariable(scale_factors_.data());
            } else {
              LOG(INFO) << "Optimization Finished... " << std::endl;
              PrintResults();

// Print the covariance matrix.
#if not defined ANDROID&& defined COMPUTE_VICALIB_COVARIANCE
              {
                std::stringstream ss;
                for (const std::string& string : covariance_names_) {
                  ss << string << " ";
                }
              }

              Eigen::MatrixXd covariance = GetSolutionCovariance(problem_);
#endif  // ANDROID

              is_finished_ = true;
            }

            LOG(INFO) << "bw_ba= " << biases_.transpose() << std::endl;
            LOG(INFO) << "sfw_sfa= " << scale_factors_.transpose() << std::endl;
            LOG(INFO) << "G= " << imu_.g_.transpose() << std::endl;
            LOG(INFO) << "ts= " << imu_.time_offset_ << std::endl;
            break;
          } else if (summary.termination_type != ceres::NO_CONVERGENCE) {
            is_finished_ = true;
          }
        }
        catch (const std::exception& e) {
          LOG(WARNING) << e.what() << std::endl;
        }
      }
    }

    is_running_ = false;
  }

  pthread_mutex_t update_mutex_;
  pthread_attr_t thread_attr_;
  pthread_t thread_;
  bool should_run_;
  bool is_running_;

  bool fix_intrinsics_;

  std::shared_ptr<ceres::Problem> problem_;
  std::vector<std::shared_ptr<VicalibFrame<double> > > t_wk_;
  std::vector<std::unique_ptr<CameraAndPose> > cameras_;
  std::vector<std::vector<std::unique_ptr<calibu::CostFunctionAndParams> > >
      proj_costs_;
  std::vector<std::unique_ptr<ImuCostFunctionAndParams> > imu_costs_;
  std::vector<double*> covariance_params_;
  std::vector<double> camera_proj_rmse_;
  std::vector<std::string> covariance_names_;
  std::vector<std::vector<ceres::ResidualBlockId> > projection_residuals_;

  ceres::Problem::Options prob_options_;
  ceres::Solver::Options solver_options_;

  ceres::LossFunctionWrapper loss_func_;
  LocalParamSe3 local_param_se3_;
  LocalParamSo3 local_param_so3_;
  InterpolationBufferT<ImuMeasurementT, double> imu_buffer_;
  ImuCalibrationT<double> imu_;
  Vector6d biases_;
  double gyro_sigma_;
  double accel_sigma_;
  Vector6d scale_factors_;
  ceres::CauchyLoss imu_loss_func_;
  unsigned int num_imu_residuals_;
  unsigned int num_iterations_;
  bool is_bias_active_;
  bool is_scale_factor_active_;
  bool is_inertial_active_;
  bool is_visual_active_;
  bool optimize_rotation_only_;
  bool is_finished_;
  bool is_gravity_initialized_;
  double mse_;
  bool optimize_time_offset_;
};
}  // namespace visual_inertial_calibration
#endif  // VISUAL_INERTIAL_CALIBRATION_VICALIBRATOR_H_
