#include "testing_common.h"
#include "gtest/gtest.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vicalib/vicalib-engine.h>


TEST(ViTests, ViSimTest) {
  std::cout << "Running test: Vi Sim" << std::endl;
  // Convert the relative path to the resources to an absolute path.
  char resolved_path[4096];
  realpath("vi_sim_resources", resolved_path);
  std::cerr << "Path resolved to " << resolved_path << std::endl;
  std::string uri =
      "-models poly3 -cam file://<RESOURCE_DIR>/images/*.pgm "
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
}
