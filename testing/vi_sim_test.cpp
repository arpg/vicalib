#include "testing_common.h"
#include "gtest/gtest.h"
#include <gflags/gflags.h>
#include <sophus/se3.hpp>
#include <glog/logging.h>
#include <vicalib/vicalib-engine.h>
#define LOGDIFF_TOLERANCE 1e-3
#define REPROJ_TOLERANCE 1e-1
#define CAMERA_TOLERANCE 5
#define TIMEOFFSET_TOLERANCE 1e-4

TEST(ViTests, ViSimTest) {
  std::cout << "Running test: Vi Sim" << std::endl;
  // Convert the relative path to the resources to an absolute path.
  char resolved_path[4096];
  realpath("vi_sim_resources", resolved_path);
  std::cerr << "Path resolved to " << resolved_path << std::endl;
  std::string uri =
      "-models linear -cam file://<RESOURCE_DIR>/images/*.pgm "
      "-imu csv://<RESOURCE_DIR>/imu -nouse_only_when_static "
      "-nohas_initial_guess -grid_preset medium -noexit_vicalib_on_finish";
  //-alsologtostderr


  size_t index = 0;
  while (true) {
    index = uri.find("<RESOURCE_DIR>", index);
    if (index == std::string::npos) break;
    uri.replace(index, 14, std::string(resolved_path));
    index += 3;
  }

  std::vector<std::string> strings;
  strings.push_back("");
  std::istringstream f(uri);
  std::string s;
  while (getline(f, s, ' ')) {
      strings.push_back(s);
  }


  char** argv;
  int argc = strings.size();

  // FIGURE OUT HOW TO DELETE THESE AS GFLAGS CHANGES THINGS
  argv = new char*[argc];
  for (int ii = 0; ii < argc; ++ii) {
    argv[ii] = new char[strings[ii].length()];
    strcpy(argv[ii], strings[ii].c_str());
  }

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  LOG(INFO) << "Starting vicalib engine.";
  visual_inertial_calibration::VicalibEngine engine(
      [](){}, [](const std::shared_ptr<
                 visual_inertial_calibration::CalibrationStats>&){});
  engine.Run();
  std::shared_ptr<visual_inertial_calibration::CalibrationStats> stats =
          engine.stats();

/*
  // PRINT OUT THE TRANSFER MATRIX T_ck PER CAMERA
  for (size_t ii = 0; ii < stats->t_ck_vec.size(); ++ii) {
      std::cout << "The final T_ck_" << ii << " is: " <<
                   std::endl << stats->t_ck_vec[ii].matrix() << std::endl;
  }
*/

  Eigen::Matrix4d t_ck_ground_truth;
  t_ck_ground_truth <<  0, 	1, 	0,	0,
                        0,	0,	1,	0,
                        1,	0,	0,	0,
                        0,	0,	0,	1;

  Eigen::Vector4d cam_intrinsics_ground_truth;
  cam_intrinsics_ground_truth << 335.639853151,	335.639853151,	400,	300;

  for (size_t ii = 0; ii < stats->t_ck_vec.size(); ++ii) {
      Sophus::SE3d diff = Sophus::SE3d(t_ck_ground_truth) *
              stats->t_ck_vec[ii].inverse();
      EXPECT_LT(diff.log().norm(), LOGDIFF_TOLERANCE)
              << "T_ck tolerance for camera " << ii << " too high.";
      EXPECT_LT(stats->reprojection_error[ii], REPROJ_TOLERANCE)
              << "Reprojection error for camera " << ii
              << " too high.";
      EXPECT_LT((stats->cam_intrinsics[ii] -
                 cam_intrinsics_ground_truth).norm(), CAMERA_TOLERANCE)
              << "Parameter error too high for camera " << ii
              << ". Estimate: " << stats->cam_intrinsics[ii].transpose()
              << ". Ground truth: " << cam_intrinsics_ground_truth.transpose();
      EXPECT_LT(stats->ts, TIMEOFFSET_TOLERANCE)
              << "Time offset is incorrect.";
  }
}
