// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.
#include <vicalib/vicalib.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vicalib/vicalib-engine.h>


static const char* g_usage =
"Examples: \n\n"
" vicalib -grid_preset=letter -cam rectify:[file=cameras.xml]//deinterlace://uvc://\n"
" vicalib -grid_preset=letter -cam file:///home/user/data/*.pgm\n";

int vicalib(int argc, char *argv[], unsigned char * json_data, int json_data_length, double reprojection_errors[2])
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
  engine.Run(json_data, json_data_length);

  auto stats = engine.stats();

  reprojection_errors[0] = stats->reprojection_error[0];
  reprojection_errors[1] = stats->reprojection_error[1];

  google::FlushLogFiles(google::GLOG_INFO);
  google::ShutdownGoogleLogging();

  return 0;
}
