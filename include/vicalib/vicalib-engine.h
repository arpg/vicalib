// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef VISUAL_INERTIAL_CALIBRATION_VICALIB_ENGINE_H_
#define VISUAL_INERTIAL_CALIBRATION_VICALIB_ENGINE_H_

#include <memory>

#include <gflags/gflags.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <vicalib/calibration-stats.h>
#include <vicalib/eigen-alignment.h>
#include <vicalib/boxcar-filter.h>

DECLARE_string(grid_preset);  // Defined in vicalib-engine.cc.
DECLARE_int32(static_threshold_preset);  // Defined in vicalib-engine.cc.
DECLARE_bool(use_static_threshold_preset);  // Defined in vicalib-engine.cc.

namespace visual_inertial_calibration {

enum GridPreset {
  GridPresetGWUSmall  = 0,  // 19x10 grid at GWU
  GridPresetGoogleLarge = 1,  // 25x36 from Google folks
  GridPresetMedium = 2,     // 18x25 grid at CU
};

enum StaticThresholdPreset {
  StaticThresholdManual = 0,  // relaxed threshold (handheld calibration)
  StaticThresholdStrict = 1,  // tight threshold (automatic calibration)
};

class VicalibTask;

// Handles input and configuration of the visual-inertial calibration process.
class VicalibEngine {
 public:
  VicalibEngine(const std::function<void()>& stop_sensors_callback,
                const std::function<void(
                    const std::shared_ptr<CalibrationStats>&)>&
                update_stats_callback);
  VicalibEngine(const VicalibEngine&) = default;
  ~VicalibEngine();

  void Run();
  bool CameraLoop();
  void ImuHandler(const hal::ImuMsg& imu);
  std::shared_ptr<CalibrationStats> stats() { return stats_; }

 protected:
  std::shared_ptr<VicalibTask> InitTask();
  void WriteCalibration();  
  void CalibrateAndDrawLoop();
  bool SeenEnough() const;
  void CreateGrid();

 private:
  std::shared_ptr<VicalibTask> vicalib_;
  double first_imu_time_;
  int frames_skipped_;
  std::function<void()> stop_sensors_callback_;
  std::function<void(const std::shared_ptr<CalibrationStats>&)>
  update_stats_callback_;
  std::shared_ptr<CalibrationStats> stats_;
  bool sensors_finished_;
  std::unique_ptr<hal::Camera> camera_;
  std::unique_ptr<hal::IMU> imu_;
  BoxcarFilter<Eigen::Vector3d> gyro_filter_, accel_filter_;
  Eigen::MatrixXi grid_;
  double grid_spacing_;
};
}  // namespace visual_inertial_calibration
#endif  // VISUAL_INERTIAL_CALIBRATION_VICALIB_ENGINE_H_
