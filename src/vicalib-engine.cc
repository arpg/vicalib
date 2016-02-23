// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <time.h>
#include <thread>
#include <functional>
#include <unistd.h>

#include <fstream>

#include <calibu/cam/camera_xml.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/target/RandomGrid.h>
#include <calibu/target/GridDefinitions.h>

#include <HAL/Camera/CameraDevice.h>
#include <glog/logging.h>
#include <HAL/Messages/Matrix.h>

#include <vicalib/eigen-alignment.h>
#include <vicalib/vicalib-task.h>
#include <vicalib/vicalib-engine.h>
#include <vicalib/vicalibrator.h>
#include <vicalib/calibration-stats.h>

static const int64_t kCalibrateAllPossibleFrames = -1;

#ifdef BUILD_GUI
DEFINE_bool(paused, false, "Start video paused");
#endif

DECLARE_bool(use_system_time); // Defined in vicalib-task.cc

DEFINE_bool(calibrate_imu, true,
            "Calibrate the IMU in addition to the camera.");
DEFINE_bool(calibrate_intrinsics, true,
            "Calibrate the camera intrinsics as well as the extrinsics.");
DEFINE_string(device_serial, "-1", "Serial number of device.");
DEFINE_bool(save_poses,false, "Save calibrated camera poses when done.");
DEFINE_bool(exit_vicalib_on_finish, true,
            "Exit vicalib when the optimization finishes.");
DEFINE_int32(frame_skip, 0, "Number of frames to skip between constraints.");
DEFINE_int32(grid_height, 10, "Height of grid in circles.");
DEFINE_int32(grid_width, 19, "Width of grid in circles.");
DEFINE_double(grid_spacing, 0.01355/*0.254 / (19 - 1) meters */,
              "Distance between circles on grid (m).");
DEFINE_int32(grid_seed, 71, "seed used to generate the grid.");
DEFINE_bool(has_initial_guess, false,
            "Whether or not the given calibration file has a valid guess.");
DEFINE_bool(output_conics, false,
            "Output center of found conics to the file ./conics.csv.");
DEFINE_string(grid_preset, "",
             "Which grid preset to use. "
             "Use \"small\" for small GWU grid, \"large\" for large Google grid, or \"letter\" for the CU grid.");
DEFINE_double(max_reprojection_error, 0.15,  // pixels
              "Maximum allowed reprojection error.");
DEFINE_int64(num_vicalib_frames, kCalibrateAllPossibleFrames,
             "Number of frames to process before calibration begins.");
DEFINE_bool(print_poses, false, "Output poses to poses.txt");
DEFINE_string(output, "cameras.xml",
              "Output XML file to write camera models to.");
DEFINE_string(output_log_file, "vicalibrator.log",
              "Calibration result output log file.");
DEFINE_bool(scaled_ir_depth_cal, false,
            "Produce ir and depth calibration by rescaling RGB.");
DEFINE_double(static_accel_threshold, 0.08, "Threshold for acceleration at "
              "which we consider the device static.");
DEFINE_double(static_gyro_threshold, 0.04, "Threshold for angular velocity at "
              "which we consider the device static.");
DEFINE_int32(static_threshold_preset,
             visual_inertial_calibration::StaticThresholdManual,
             "Which grid preset to use. "
             "Must be a visual_inertial_calibration::StaticThresholdPreset.");
DEFINE_bool(use_only_when_static, false, "Only use frames where the device is "
            "stationary.");
DEFINE_bool(use_static_threshold_preset, false,
            "Use one of the predefined static thresholds.");
DEFINE_string(cam, "", "Camera URI");
DEFINE_string(imu, "", "IMU URI (if available)");
DEFINE_string(models, "",
              "Comma-separated list of camera model types to calibrate. "
              "Must be in channel order.");
DEFINE_string(model_files, "",
              "Comma-separated list of camera model files pre-load. "
              "If specified this supercedes the 'models' flag."
              "Must be in channel order.");
DEFINE_string(output_pattern_file, "",
              "Output EPS or SVG file to save the calibration pattern.");
DEFINE_double(grid_large_rad, 0.00423,
              "Radius of large dots (m) (necessary to save the pattern).");
DEFINE_double(grid_small_rad, 0.00283,
              "Radius of small dots (m) (necessary to save the pattern).");
DEFINE_int32(max_iters, 200, "Max iterations.");

DEFINE_double(gyro_sigma, IMU_GYRO_SIGMA,
              "Sigma of gyroscope measurements.");
DEFINE_double(accel_sigma, IMU_ACCEL_SIGMA,
              "Sigma of accel measurements.");

namespace visual_inertial_calibration {

static const timespec sleep_length = {0, 30000000};  // 30 ms

using std::placeholders::_1;

VicalibEngine::VicalibEngine(const std::function<void()>& stop_sensors_callback,
                             const std::function<void(
                                 const std::shared_ptr<CalibrationStats>&)>&
                             update_stats_callback) :
    first_imu_time_(DBL_MAX),
    frames_skipped_(0),
    stop_sensors_callback_(stop_sensors_callback),
    update_stats_callback_(update_stats_callback),
    sensors_finished_(false),
    gyro_filter_(10, FLAGS_static_gyro_threshold),
    accel_filter_(10, FLAGS_static_accel_threshold) {

  if (!FLAGS_cam.empty()) {
    try {
      camera_.reset(new hal::Camera(hal::Uri(FLAGS_cam)));
    } catch (std::exception& ex) {
      LOG(FATAL) << "Could not create camera from URI: " << FLAGS_cam
                 << ". Reason: " << ex.what();
    }
    stats_.reset(new CalibrationStats(camera_->NumChannels()));
  }

  if (!FLAGS_imu.empty()) {
    try {
      imu_.reset(new hal::IMU(FLAGS_imu));
      imu_->RegisterIMUDataCallback(
          std::bind(&VicalibEngine::ImuHandler, this, _1));
    } catch (std::exception& ex) {
      LOG(FATAL) << "Could not create IMU from URI: " << FLAGS_imu
                 << ". Reason: " << ex.what();
    }
  } else
    FLAGS_calibrate_imu = false;
}

VicalibEngine::~VicalibEngine() {
  if (vicalib_) {
    vicalib_->Finish(FLAGS_output);
  }
}

std::shared_ptr<VicalibTask> VicalibEngine::InitTask() {
  std::vector<size_t> widths, heights;
  for (size_t i = 0; i < camera_->NumChannels(); ++i) {
    widths.push_back(camera_->Width(i));
    heights.push_back(camera_->Height(i));
    LOG(INFO) << "Camera " << i << " with width: " << camera_->Width(i) <<
                 " height: " << camera_->Height(i) << std::endl;
  }

  std::vector<std::string> model_files;
  {
    std::stringstream ss(FLAGS_model_files);
    std::string item;
    while (std::getline(ss, item, ',')) {
      model_files.push_back(item);
    }
  }

  std::vector<std::string> model_strings;
  {
    std::stringstream ss(FLAGS_models);
    std::string item;
    while (std::getline(ss, item, ',')) {
      model_strings.push_back(item);
    }
  }

  if( model_files.empty() && model_strings.size() < camera_->NumChannels()) {
    LOG(INFO) << "Only " << model_strings.size() <<
      " models declared using 'model_strings'.  Need one for all the "
      <<  camera_->NumChannels() << " channels; assuming poly3";
    model_strings.resize(camera_->NumChannels(), "poly3");
  }

  // use model xml files, if provided
  aligned_vector<CameraAndPose> input_cameras;
  if( !model_files.empty() ){
    for (size_t i = 0; i < model_files.size(); ++i) {
      std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRig(model_files[i]);
      input_cameras.emplace_back( rig->cameras_[0], Sophus::SE3d());
      LOG(INFO) << "Initalizing with user provided model file: " 
        <<  model_files[i] << "\n" ;
    }
  }
  else{ 
    for (size_t i = 0; i < model_strings.size(); ++i) {
      const std::string& type = model_strings[i];
      int w = camera_->Width(i);
      int h = camera_->Height(i);

      if (type == "fov") {
        Eigen::Vector2i size_;
        Eigen::VectorXd params_(calibu::FovCamera<double>::NumParams);
        size_ << w, h;
        params_ << 300, 300, w/2.0, h/2.0, 0.2;
        std::shared_ptr<calibu::CameraInterface<double>>
            starting_cam(new calibu::FovCamera<double>(params_, size_));
        starting_cam->SetType("calibu_fu_fv_u0_v0_w");
        input_cameras.emplace_back(starting_cam, Sophus::SE3d());

      } else if (type == "poly2") {
        Eigen::Vector2i size_;
        Eigen::VectorXd params_(calibu::Poly2Camera<double>::NumParams);
        size_ << w, h;
        params_ << 300, 300, w/2.0, h/2.0, 0.0, 0.0;
        std::shared_ptr<calibu::CameraInterface<double>>
            starting_cam(new calibu::Poly2Camera<double>(params_, size_));
        starting_cam->SetType("calibu_fu_fv_u0_v0_k1_k2");
        input_cameras.emplace_back(starting_cam, Sophus::SE3d());

      } else if (type == "poly3" || type =="poly") {
        Eigen::Vector2i size_;
        Eigen::VectorXd params_(calibu::Poly3Camera<double>::NumParams);
        size_ << w, h;
        params_ << 300, 300, w/2.0, h/2.0, 0.0, 0.0, 0.0;
        std::shared_ptr<calibu::CameraInterface<double>>
            starting_cam(new calibu::Poly3Camera<double>(params_, size_));
        starting_cam->SetType("calibu_fu_fv_u0_v0_k1_k2_k3");
        input_cameras.emplace_back(starting_cam, Sophus::SE3d());

      } else if (type == "rational6" || type == "rational") {
		Eigen::Vector2i size_;
		Eigen::VectorXd params_(calibu::Rational6Camera<double>::NumParams);
		size_ << w, h;
		params_  << 300, 300, w/2.0, h/2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		std::shared_ptr<calibu::CameraInterface<double>>
		starting_cam(new calibu::Rational6Camera<double>(params_, size_));
		starting_cam->SetType("calibu_fu_fv_u0_v0_rational6");
		input_cameras.emplace_back(starting_cam, Sophus::SE3d());

      } else if (type == "kb4") {
		Eigen::Vector2i size_;
		Eigen::VectorXd params_(calibu::KannalaBrandtCamera<double>::NumParams);
		size_ << w, h;
		params_  << 300, 300, w/2.0, h/2.0, 0.0, 0.0, 0.0, 0.0;
		std::shared_ptr<calibu::CameraInterface<double>>
		  starting_cam(new calibu::KannalaBrandtCamera<double>(params_, size_));
		starting_cam->SetType("calibu_fu_fv_u0_v0_kb4");
		input_cameras.emplace_back(starting_cam, Sophus::SE3d());

      } else if (type == "linear") {
        Eigen::Vector2i size_;
        Eigen::VectorXd params_(calibu::LinearCamera<double>::NumParams);
        size_ << w, h;
        params_ << 300, 300, w/2.0, h/2.0;
        std::shared_ptr<calibu::CameraInterface<double>>
          starting_cam(new calibu::LinearCamera<double>(params_, size_));
        starting_cam->SetType("calibu_fu_fv_u0_v0");
        input_cameras.emplace_back(starting_cam, Sophus::SE3d());
      }
      input_cameras.back().camera->SetRDF(calibu::RdfRobotics.matrix());
    }
  }

  std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
  camera_->Capture(*images);
  for (size_t i = 0; i < images->Size() && i < input_cameras.size(); ++i) {
    input_cameras[i].camera->SetSerialNumber(images->at(i)->SerialNumber());
  }

  Vector6d biases(Vector6d::Zero());
  Vector6d scale_factors(Vector6d::Ones());

  if (FLAGS_use_static_threshold_preset) {
    switch (FLAGS_static_threshold_preset) {
      case StaticThresholdManual:
        FLAGS_static_accel_threshold = 0.09;
        FLAGS_static_gyro_threshold = 0.05;
        break;
      case StaticThresholdStrict:
        FLAGS_static_accel_threshold = 0.05;
        FLAGS_static_gyro_threshold = 0.025;
        break;
      default:
        LOG(FATAL) << "Unknown static threshold preset "
                   << FLAGS_static_threshold_preset;
        break;
    }
  }

  std::vector<double> max_errors(camera_->NumChannels(),
                                 FLAGS_max_reprojection_error);
  std::shared_ptr<VicalibTask> task(
      new VicalibTask(camera_->NumChannels(), widths, heights,
                      grid_spacing_, grid_,
                      !FLAGS_calibrate_intrinsics,
                      input_cameras, max_errors));

  task->GetCalibrator().SetSigmas(FLAGS_gyro_sigma, FLAGS_accel_sigma);
  task->GetCalibrator().SetBiases(biases);
  task->GetCalibrator().SetScaleFactor(scale_factors);

#ifdef BUILD_GUI
  pangolin::RegisterKeyPressCallback(' ', [&]() {
      FLAGS_paused = !FLAGS_paused;
    });
  pangolin::RegisterKeyPressCallback('[', [&]() {
      FLAGS_num_vicalib_frames = 0; // this starts the calibration
    });
#endif  // BUILD_GUI
  return task;
}

void Draw(const std::shared_ptr<VicalibTask>& task) {
  task->Draw();
#ifdef BUILD_GUI
  pangolin::FinishFrame();
#endif
}

inline Eigen::Matrix<double, 6, 1> _T2Cart(const Eigen::Matrix4d& T) {
  Eigen::Matrix<double, 6, 1> Cart;
  Eigen::Matrix<double, 3, 3> R = T.block<3, 3>(0, 0);
  Eigen::Vector3d rpq;
  // roll
  rpq[0] = atan2(R(2, 1), R(2, 2));

  // pitch
  double det = -R(2, 0) * R(2, 0) + 1.0;
  if (det <= 0) {
    if (R(2, 0) > 0) {
      rpq[1] = -M_PI / 2.0;
    } else {
      rpq[1] = M_PI / 2.0;
    }
  } else {
    rpq[1] = -asin(R(2, 0));
  }

  // yaw
  rpq[2] = atan2(R(1, 0), R(0, 0));

  Cart[0] = T(0, 3);
  Cart[1] = T(1, 3);
  Cart[2] = T(2, 3);
  Cart[3] = rpq[0];
  Cart[4] = rpq[1];
  Cart[5] = rpq[2];

  return Cart;
}

void VicalibEngine::WriteCalibration() {
  vicalib_->GetCalibrator().WriteCameraModels(FLAGS_output);
  if (FLAGS_print_poses) {
    FILE* f = fopen("poses.txt", "w");
    Eigen::Matrix<double, 6, 1> pose;

    std::vector<bool> good_frames = vicalib_->GetGoodFrames( );

    for (int ii = 0; ii < good_frames.size(); ii++) {
      if (good_frames[ii]) {
        std::shared_ptr<VicalibFrame<double> > frame = vicalib_->GetCalibrator().GetFrame(ii);
        pose = _T2Cart(frame->t_wp_.matrix());
        fprintf(f, "%f\t%f\t%f\t%f\t%f\t%f\n",
                pose(0), pose(1), pose(2), pose(3), pose(4), pose(5));
      }
    }
    fclose(f);
  }
}

void VicalibEngine::CalibrateAndDrawLoop() {
  LOG(INFO) << "Entering calibration loop.";
  if (!vicalib_->IsRunning()) {
    vicalib_->Start(FLAGS_has_initial_guess);
  }

  // Wait for the thread to start
  while (!vicalib_->IsRunning()) {
    usleep(500);
  }

  stats_->status = CalibrationStats::StatusOptimizing;
  bool finished = false;
  while (true) {
    stats_->total_mse = vicalib_->GetMeanSquaredError();
    stats_->reprojection_error = vicalib_->GetCalibrator().GetCameraProjRMSE();
    stats_->num_iterations = vicalib_->GetCalibrator().GetNumIterations();
    stats_->ts = vicalib_->GetCalibrator().time_offset();
    stats_->t_ck_vec.resize(vicalib_->GetCalibrator().NumCameras());
    stats_->cam_intrinsics.resize(vicalib_->GetCalibrator().NumCameras());
    for (size_t ii = 0; ii < vicalib_->GetCalibrator().NumCameras(); ++ii) {
       stats_->t_ck_vec[ii] = vicalib_->GetCalibrator().GetCamera(ii).T_ck;
       stats_->cam_intrinsics[ii] =
               vicalib_->GetCalibrator().GetCamera(ii).camera->GetParams();
    }
    update_stats_callback_(std::make_shared<CalibrationStats>(*stats_));

    if (!finished && !vicalib_->IsRunning()) {
      LOG(INFO) << "Finished... " << std::endl;
      stats_->status = vicalib_->IsSuccessful() ?
          CalibrationStats::StatusSuccess : CalibrationStats::StatusFailure;
      vicalib_->Finish(FLAGS_output);
      WriteCalibration();

      if( FLAGS_save_poses ){
        std::ofstream file("poses.csv");
        file << "\% Pose file generated with vicalib.\n"
             << "\% Each line is the 12 elements from the top 3 rows of a 4x4"
             << "transformation matrix, printed row major.\n";

        for( size_t ii = 0; ii < vicalib_->GetCalibrator().NumFrames(); ii++ ){
          Eigen::Matrix4d t_wk =
            vicalib_->GetCalibrator().GetFrame(ii)->t_wp_.matrix();
          file << t_wk.row(0) << "     " << t_wk.row(1) 
            << "     " << t_wk.row(2) << std::endl;
        }
        file.close();
      }

      finished = true;
      if (FLAGS_exit_vicalib_on_finish) {
        exit(0);
      } 
    }
    Draw(vicalib_);

    nanosleep(&sleep_length, NULL);
  }
}

void VicalibEngine::Run() {
  CreateGrid();
  if (!camera_) {
    if (!FLAGS_output_pattern_file.empty())
      if (FLAGS_exit_vicalib_on_finish) {
        exit(1);
      } else {
        return;
      }
    else
      LOG(FATAL) << "No camera URI given";
  }
  while (CameraLoop() && !vicalib_->IsRunning() && !SeenEnough()) {}
  stop_sensors_callback_();
  sensors_finished_ = true;
  CalibrateAndDrawLoop();
}

void VicalibEngine::CreateGrid() {
  double large_rad = FLAGS_grid_large_rad;
  double small_rad = FLAGS_grid_small_rad;
  Eigen::Vector2d offset(0,0);
  grid_spacing_ = FLAGS_grid_spacing;

  if (FLAGS_grid_preset.empty()) {
    grid_ = calibu::MakePattern(
        FLAGS_grid_height, FLAGS_grid_width, FLAGS_grid_seed);
  }
  else {
    calibu::LoadGridFromPreset(FLAGS_grid_preset,grid_,grid_spacing_,large_rad,small_rad);
  }

  if (!FLAGS_output_pattern_file.empty()) {
    // eps or svg?
    std::string::size_type p = FLAGS_output_pattern_file.find_last_of('.');
    bool eps =  (p != std::string::npos &&
        p + 3 < FLAGS_output_pattern_file.size() &&
        (FLAGS_output_pattern_file[p+1] == 'e' ||
        FLAGS_output_pattern_file[p+1] == 'E') &&
        (FLAGS_output_pattern_file[p+2] == 'p' ||
        FLAGS_output_pattern_file[p+2] == 'P') &&
        (FLAGS_output_pattern_file[p+3] == 's' ||
        FLAGS_output_pattern_file[p+3] == 'S'));
    if (eps) {
      const double pts_per_unit = 72. / 2.54 * 100.; // points per meter
      calibu::TargetGridDot(grid_spacing_, grid_).
          SaveEPS(FLAGS_output_pattern_file, offset, small_rad, large_rad,
                  pts_per_unit);
    } else {
      calibu::TargetGridDot(grid_spacing_, grid_).
          SaveSVG(FLAGS_output_pattern_file, small_rad, large_rad);
    }
    LOG(INFO) << "File " << FLAGS_output_pattern_file << " saved";
    if (!eps) {
      LOG(INFO) << "You may convert the file into PDF with ImageMagick:"
                << std::endl
                << "convert -density 300 " << FLAGS_output_pattern_file << " "
                << FLAGS_output_pattern_file << ".pdf";
    }
  }
}

bool VicalibEngine::CameraLoop() {
  if (!vicalib_) {
    vicalib_ = InitTask();

    if (!vicalib_) {
      LOG(WARNING) << "Vicalib task still NULL. Skipping frame.";
      return false;
    }
  }

#ifdef BUILD_GUI
  while (FLAGS_paused) {
    Draw(vicalib_);
    nanosleep(&sleep_length, NULL);
  }
#endif

  std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
  bool captured = camera_->Capture(*images);
  if (captured) {
    cv::Mat temp_mat;
    for (int ii = 0; ii < images->Size(); ++ii) {
      std::shared_ptr<hal::Image> img = images->at(ii);
      if (img->Mat().channels() == 3) {
        cv::cvtColor(img->Mat(), temp_mat, CV_BGR2GRAY);
      }
      memcpy((void*)img->data(), temp_mat.data,
             temp_mat.elemSize() * temp_mat.rows * temp_mat.cols);
      temp_mat.copyTo(img->Mat());
    }
  }

  bool should_use = true;
  if (FLAGS_use_only_when_static) {
    should_use = accel_filter_.IsStable() && gyro_filter_.IsStable();
  }

  const double frame_timestamp = FLAGS_use_system_time ?
        images->Ref().system_time() : images->Timestamp();
  if (frames_skipped_ >= FLAGS_frame_skip &&
      (first_imu_time_ == DBL_MAX || frame_timestamp > first_imu_time_)) {
    if (captured && should_use) {
      frames_skipped_ = 0;
      std::vector<bool> valid_frames = vicalib_->AddSuperFrame(images);
      for (size_t ii = 0; ii < valid_frames.size() ; ++ii) {
        if (valid_frames[ii]) {
          ++stats_->num_frames_processed[ii];
        }
      }
    }
  } else {
    ++frames_skipped_;
  }

  stats_->status = CalibrationStats::StatusCapturing;
  update_stats_callback_(std::make_shared<CalibrationStats>(*stats_));
  Draw(vicalib_);
  return captured;
}

void VicalibEngine::ImuHandler(const hal::ImuMsg& imu) {
  CHECK(imu.has_accel());
  CHECK(imu.has_gyro());

  if(!vicalib_ || sensors_finished_) {
    return;
  }

  Eigen::VectorXd gyro, accel;
  ReadVector(imu.accel(), &accel);
  ReadVector(imu.gyro(), &gyro);
  accel_filter_.Add(accel);
  gyro_filter_.Add(gyro);

  if (first_imu_time_ == DBL_MAX) {
    first_imu_time_ = FLAGS_use_system_time ? imu.system_time() :
                                              imu.device_time();
  }

  vicalib_->AddIMU(imu);
}

bool VicalibEngine::SeenEnough() const {
  return ((FLAGS_num_vicalib_frames != kCalibrateAllPossibleFrames &&
           *std::max_element(stats_->num_frames_processed.begin(),
                             stats_->num_frames_processed.end()) >=
           FLAGS_num_vicalib_frames) ||
          sensors_finished_);
}
}  // namespace visual_inertial_calibration
