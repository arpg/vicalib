// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vicalib/vicalib-engine.h>

static const char* g_usage =
"Examples: \n\n"
" vicalib -grid_preset=letter -cam rectify:[file=cameras.xml]//deinterlace://uvc://\n"
" vicalib -grid_preset=letter -cam file:///home/user/data/*.pgm\n";

int main(int argc, char *argv[]) 
{
  if( argc == 1 ){
    google::SetUsageMessage(g_usage);
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
