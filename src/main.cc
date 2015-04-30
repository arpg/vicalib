// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vicalib/vicalib-engine.h>

#define USAGE \

int main(int argc, char *argv[]) 
{
  if( argc == 1 ){
    google::SetUsageMessage("Vicalib options:");
    google::ShowUsageWithFlags(argv[0]);
    return -1;
  }
  
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  LOG(INFO) << "Starting vicalib engine.";
  visual_inertial_calibration::VicalibEngine engine(
      [](){}, [](const std::shared_ptr<
                 visual_inertial_calibration::CalibrationStats>&){});
  engine.Run();
 
  return 0;
}
