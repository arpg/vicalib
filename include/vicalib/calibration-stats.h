// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef VISUAL_INERTIAL_CALIBRATION_CALIBRATION_STATS_H_
#define VISUAL_INERTIAL_CALIBRATION_CALIBRATION_STATS_H_

#include <string>
#include <vector>
#include <sophus/se3.hpp>
#include <Eigen/Eigen>

namespace visual_inertial_calibration {

// A structure to capture the on-going calibration processing details.
struct CalibrationStats {
  enum CalibrationStatus {
    StatusInactive,
    StatusCapturing,
    StatusOptimizing,
    StatusSuccess,
    StatusFailure
  };

  explicit CalibrationStats(int nstreams) : cam_names(nstreams),
                                            num_frames_processed(nstreams, 0),
                                            reprojection_error(nstreams, 0),
                                            ts(0),
                                            t_ck_vec(nstreams, Sophus::SE3d()),
                                            cam_intrinsics(nstreams,
                                                           Eigen::VectorXd()),
                                            total_mse(0),
                                            status(StatusInactive),
                                            num_iterations(0) { }
  std::vector<std::string> cam_names;
  std::vector<int> num_frames_processed;
  std::vector<double> reprojection_error;
  double ts;
  std::vector<Sophus::SE3d> t_ck_vec;
  std::vector<Eigen::VectorXd> cam_intrinsics;
  double total_mse;
  CalibrationStatus status;
  int num_iterations;
};
}  // namespace visual_inertial_calibration
#endif  // VISUAL_INERTIAL_CALIBRATION_CALIBRATION_STATS_H_
