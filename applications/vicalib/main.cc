// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vicalib/vicalib-engine.h>

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "Starting vicalib engine.";
  visual_inertial_calibration::VicalibEngine engine(
      [](){}, [](const std::shared_ptr<
                 visual_inertial_calibration::CalibrationStats>&){});
  engine.Run();
  return 0;
}
