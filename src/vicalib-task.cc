#include <vicalib/vicalib-task.h>
#include <vicalib/eigen-alignment.h>

#include <time.h>
#include <random>
#include <cvars/CVar.h>
#include <calibu/conics/ConicFinder.h>
#include <calibu/pose/Pnp.h>
#include <HAL/Messages/Matrix.h>

#ifdef HAVE_PANGOLIN
#include <calibu/gl/Drawing.h>
#include <pangolin/gldraw.h>
#endif  // HAVE_PANGOLIN

DECLARE_bool(calibrate_imu);      // Defined in vicalib-engine.cc
DECLARE_bool(has_initial_guess);  // Defined in vicalib-engine.cc.
DECLARE_bool(output_conics);			// Defined in vicalib-engine.cc.
DEFINE_bool(clip_good, false, "Output proto file of only good tracked images");
DECLARE_string(imu);              // Defined in vicalib-engine.cc.
DEFINE_bool(find_time_offset, true,
            "Optimize for the time offset between the IMU and images");
DEFINE_double(function_tolerance, 1e-6,
              "Convergence criteria for the optimizer.");

DEFINE_double(max_fx_diff, 10.0, "Maximum fx difference between calibrations.");
DEFINE_double(max_fy_diff, 10.0, "Maximum fy difference between calibrations.");
DEFINE_double(max_cx_diff, 10.0, "Maximum cx difference between calibrations.");
DEFINE_double(max_cy_diff, 10.0, "Maximum cy difference between calibrations.");

DEFINE_double(max_fov_w_diff, 0.3,
              "Maximum fov w parameter difference between calibrations.");

DEFINE_double(max_poly3_diff_k1, 0.1,
              "Maximum poly3 k1 difference between calibrations.");
DEFINE_double(max_poly3_diff_k2, 0.1,
              "Maximum poly3 k2 difference between calibrations.");
DEFINE_double(max_poly3_diff_k3, 0.1,
              "Maximum poly3 k3 difference between calibrations.");
DEFINE_double(max_camera_trans_diff, 0.1,
              "Max Distance between camera's estimated pose for calibrations"
              " runs.");
DEFINE_double(max_camera_angle_diff, 0.1, "Maximum angle difference in radians"
              " along axeses of camera between calibrations.");
DEFINE_double(max_imu_gyro_diff, 0.1,
              "Maximum gyroscope bias difference between calibrations.");
DEFINE_double(max_imu_accel_diff, 0.1,
              "Maximum accelrometer bias difference between calibrations.");

DEFINE_bool(use_system_time, true,
              "Whether to use device or system time for sensors.");

namespace visual_inertial_calibration {

struct VicalibGuiOptions {
#ifdef HAVE_PANGOLIN
  VicalibGuiOptions() :
      disp_mse("ui.MSE"),
      disp_frame("ui.frame"),
      disp_thresh("ui.Display Thresh", false),
      disp_lines("ui.Display Lines", true),
      disp_cross("ui.Display crosses", true),
      disp_bbox("ui.Display bbox", true),
      disp_barcode("ui.Display barcode", false) {}

  pangolin::Var<double> disp_mse;
  pangolin::Var<int> disp_frame;
  pangolin::Var<bool> disp_thresh;
  pangolin::Var<bool> disp_lines;
  pangolin::Var<bool> disp_cross;
  pangolin::Var<bool> disp_bbox;
  pangolin::Var<bool> disp_barcode;
#endif  // HAVE_PANGOLIN
};

VicalibTask::VicalibTask(
        size_t num_streams,
        const std::vector<size_t>& width,
        const std::vector<size_t>& height,
        double grid_spacing,
        const Eigen::MatrixXi& grid,
        bool fix_intrinsics,
        const aligned_vector<CameraAndPose>& input_cameras,
        const std::vector<double>& max_reproj_errors) :
    image_processing_(),
    conic_finder_(num_streams),
    target_(num_streams,
            calibu::TargetGridDot(grid_spacing, grid)),
    grid_size_(grid.cols(), grid.rows()),
    grid_spacing_(grid_spacing),
    calib_cams_(num_streams, 0),
    frame_times_(num_streams, 0),
    current_frame_time_(),
    nstreams_(num_streams),
    width_(width),
    height_(height),
    logger_(hal::Logger::GetInstance()),
    calib_frame_(-1),
    tracking_good_(num_streams, false),
    t_cw_(num_streams),
    num_frames_(0),
    calibrator_(),
    input_cameras_(input_cameras),
    max_reproj_errors_(max_reproj_errors),
    image_time_offset(-1),

#ifdef HAVE_PANGOLIN
    textures_(nstreams_),
    stacks_(),
    imu_strips_(),
    handler_(stacks_),
#endif  // HAVE_PANGOLIN
    options_(nullptr) {
  for (size_t i = 0; i < nstreams_; ++i) {
    image_processing_.emplace_back(width[i], height[i]);
    image_processing_[i].Params().black_on_white = true;
    image_processing_[i].Params().at_threshold = 0.9;
    image_processing_[i].Params().at_window_ratio = 30.0;

    conic_finder_[i].Params().conic_min_area = 4.0;
    conic_finder_[i].Params().conic_min_density = 0.6;
    conic_finder_[i].Params().conic_min_aspect = 0.2;
  }

  if (FLAGS_clip_good) {
    logger_.LogToFile("", "good_tracking");
  }
  calibrator_.FixCameraIntrinsics(fix_intrinsics);
  input_imu_biases_ = calibrator_.GetBiases();

  for (size_t ii = 0; ii < num_streams; ++ii) {
    const int w_i = width[ii];
    const int h_i = height[ii];
    if (ii < input_cameras.size()) {
      calib_cams_[ii] = calibrator_.AddCamera(input_cameras[ii].camera,
                                              input_cameras[ii].T_ck);
    } else {
      // Generic starting set of parameters.
      Eigen::Vector2i size_;
      Eigen::VectorXd params_;
      size_ << w_i, h_i;
      params_ << 300, 300, w_i / 2.0, h_i / 2.0, 0.2;
      std::shared_ptr<calibu::FovCamera<double>>
          starting_cam(new calibu::FovCamera<double>(params_, size_));

      calib_cams_[ii] = calibrator_.AddCamera( starting_cam, Sophus::SE3d());
    }
  }
  SetupGUI();
}

VicalibTask::~VicalibTask() { }

void VicalibTask::SetupGUI() 
{
#ifdef HAVE_PANGOLIN
  LOG(INFO) << "Setting up GUI for " << NumStreams() << " streams.";
  // Setup GUI
  const int panel_width = 150;
  pangolin::CreateWindowAndBind("Main", (NumStreams() + 1) *
                                width(0) / 2.0 + panel_width,
                                height(0) / 2.0);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  // Make things look prettier...
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);
  glLineWidth(1.7);

  // Pangolin 3D Render state
  // Create viewport for video with fixed aspect
  pangolin::CreatePanel("ui").SetBounds(1.0, 0.0, 0,
                                        pangolin::Attach::Pix(panel_width));

  pangolin::View& container = pangolin::Display("vicalib").SetBounds(
      1.0, 0.0, pangolin::Attach::Pix(panel_width), 1.0).SetLayout(
          pangolin::LayoutEqual);

  // Add view for each camera stream
  for (size_t c = 0; c < NumStreams(); ++c) {
    container.AddDisplay(
        pangolin::CreateDisplay().SetAspect(
            width(c) / static_cast<float>(height(c))));
  }

  // Add 3d view, attach input handler_
  pangolin::View& v3d = pangolin::Display("vicalib3d").SetAspect(
      static_cast<float>(width(0)) / height(0)).SetHandler(&handler_);
  container.AddDisplay(v3d);

  // 1,2,3,... keys hide and show viewports
  for (size_t ii = 0; ii < container.NumChildren(); ++ii) {
    pangolin::RegisterKeyPressCallback(
        '1' + ii, [&container, ii]() {
          container[ii].ToggleShow();
        });
  }

  for (size_t i = 0; i < nstreams_; ++i) {
    std::stringstream ss;
    ss << i;
    std::string istr = ss.str();
    CVarUtils::AttachCVar("proc.adaptive.threshold_" + istr,
                          &image_processing_[i].Params().at_threshold);
    CVarUtils::AttachCVar("proc.adaptive.window_ratio_" + istr,
                          &image_processing_[i].Params().at_window_ratio);
    CVarUtils::AttachCVar("proc.black_on_white_" + istr,
                          &image_processing_[i].Params().black_on_white);
    textures_[i].Reinitialise(width(i), height(i), GL_LUMINANCE8);
  }

  options_.reset(new VicalibGuiOptions);
  stacks_.SetProjectionMatrix(
      pangolin::ProjectionMatrixRDF_TopLeft(640, 480, 420, 420, 320, 240, 0.01,
                                            1E6));
  stacks_.SetModelViewMatrix(
      pangolin::ModelViewLookAtRDF(0, 0, -0.5, 0, 0, 0, 0, -1, 0));

#endif  // HAVE_PANGOLIN
}

void VicalibTask::Start(const bool has_initial_guess) {
  calibrator_.SetOptimizationFlags(has_initial_guess,
                                   has_initial_guess && FLAGS_calibrate_imu
                                   && !FLAGS_imu.empty(),
                                   !has_initial_guess,
                                   FLAGS_find_time_offset);
  calibrator_.SetFunctionTolerance(FLAGS_function_tolerance);
  calibrator_.Start();
}

bool VicalibTask::IsRunning() {
  return calibrator_.IsRunning();
}

int VicalibTask::AddFrame(double frame_time) {
  return calibrator_.AddFrame(Sophus::SE3d(Sophus::SO3d(),
                                           Eigen::Vector3d(0, 0, 1000)),
                              frame_time);
}

void VicalibTask::AddImageMeasurements(const std::vector<bool>& valid_frames) {
  size_t n = images_->Size();

  hal::Msg msg;
  msg.Clear();
  std::vector<aligned_vector<Eigen::Vector2d> > ellipses(n);
  for (size_t ii = 0; ii < n; ++ii) {
    if (!valid_frames[ii]) {      
      LOG(WARNING) << "Frame " << ii << " is invalid.";
      tracking_good_[ii] = false;
      if (ii == 0){
        good_frame_.push_back(false);
      }
      continue;
    }

    std::shared_ptr<hal::Image> img = images_->at(ii);
    image_processing_[ii].Process(img->data(),
                                  img->Width(),
                                  img->Height(),
                                  img->Width());
    conic_finder_[ii].Find(image_processing_[ii]);

    const std::vector<calibu::Conic,
                      Eigen::aligned_allocator<calibu::Conic> >& conics =
        conic_finder_[ii].Conics();
    std::vector<int> ellipse_target__map;

    tracking_good_[ii] = target_[ii].FindTarget(image_processing_[ii],
                                                conics,
                                                ellipse_target__map);
    if (!tracking_good_[ii]) {
      LOG(WARNING) << "Tracking bad for " << ii;
      continue;
    }

    if (FLAGS_clip_good && tracking_good_[ii] && (ii == 0)) {
      good_frame_.push_back(true);
      hal::ImageMsg* img_message = msg.mutable_camera()->add_image();
      img_message->set_height(img->Height());
      img_message->set_width(img->Width());
      cv::Mat image(img->Height(), img->Width(), CV_8UC1);
      memcpy(img->Mat().data, image.data, img->Height()*img->Width());
      img_message->set_data(image.data, img->Height()*img->Width());
      img_message->set_format( hal::PB_LUMINANCE );
      img_message->set_type( hal::PB_UNSIGNED_BYTE );
      if (ii == n - 1) {
        logger_.LogMessage(msg);
      }
    }


    // Print out the binary pattern of the grid in which we're interested.
    // std::ofstream("target.csv", std::ios_base::trunc) <<
    //   target_[ii].GetBinaryPattern(0) << std::endl;

    // Generate map and point structures
    for (size_t i = 0; i < conics.size(); ++i) {
      ellipses[ii].push_back(conics[i].center);
      if (FLAGS_output_conics) {
        const Eigen::Vector3d& pos_3d =
            target_[ii].Circles3D()[ellipse_target__map[i]];
       if( ellipse_target__map[i] < 0 ){
         continue;
       }

       std::cout << num_frames_ << "," << ellipse_target__map[i] <<
                        "," << conics[i].center[0] << "," <<
                        conics[i].center[1] << "," <<
                        pos_3d[0] << "," << pos_3d[1] << "," << pos_3d[2] <<
                        std::endl;
      }

    }

    // find camera pose given intrinsics
    PosePnPRansac(calibrator_.GetCamera(ii).camera, ellipses[ii],
                  target_[ii].Circles3D(), ellipse_target__map, 0, 0,
                  &t_cw_[ii]);
  }

  bool any_good = false;
  for (bool good : tracking_good_) {
    if (good) {
      any_good = true;
      break;
    }
  }
  if (!any_good) {
    LOG(WARNING) << "No well tracked frames found. Not adding frame.";
    return;
  }
  calib_frame_ = AddFrame(current_frame_time_);

  for (size_t ii = 0; ii < n; ++ii) {
    if (tracking_good_[ii]) {
      if (calib_frame_ >= 0) {
        if (ii == 0 || !tracking_good_[0]) {
          // Initialize pose of frame for least squares optimisation
          // this needs to be T_wh which is why we invert
          calibrator_.GetFrame(calib_frame_)->t_wp_ = t_cw_[ii].inverse()
              * calibrator_.GetCamera(ii).T_ck;
        }

        for (size_t p = 0; p < ellipses[ii].size(); ++p) {
          const Eigen::Vector2d pc = ellipses[ii][p];
          const Eigen::Vector2i pg = target_[ii].Map()[p].pg;

          if (0 <= pg(0) && pg(0) < grid_size_(0) && 0 <= pg(1)
              && pg(1) < grid_size_(1)) {
            const Eigen::Vector3d pg3d = grid_spacing_
                * Eigen::Vector3d(pg(0), pg(1), 0);
            // TODO(nimski): Add these correspondences in bulk to
            // avoid hitting mutex each time.
            calibrator_.AddObservation(calib_frame_, calib_cams_[ii], pg3d, pc,
                                       current_frame_time_);
          }
        }
      }
    }
  }
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


//void VicalibTask::WritePoses()
//{
//  FILE* f = fopen("poses.txt", "w");
//  for (int ii = 0; ii < t_cw_.size(); ii++) {
//    Eigen::Matrix<double, 6, 1> pose;
//    pose = _T2Cart(t_cw_[ii].inverse().matrix());
//    fprintf(f, "%f\t%f\t%f\t%f\t%f\t%f\n", pose(0), pose(1), pose(2), pose(3), pose(4), pose(5));
//  }
//  fclose(f);
//}

#ifdef HAVE_PANGOLIN

void VicalibTask::Draw3d() {
  pangolin::View& v3d = pangolin::Display("vicalib3d");
  if (!v3d.IsShown()) return;

  v3d.ActivateScissorAndClear(stacks_);
  calibu::glDrawTarget(target_[0], Eigen::Vector2d(0, 0), 1.0, 0.8, 1.0);

  if (options_->disp_barcode) {
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d> >& code_points =
        target_[0].Code3D();
    for (const Eigen::Vector3d& xwp : code_points) {
      glColor3f(1.0, 0.0, 1.0);
      pangolin::glDrawCircle(xwp.head<2>(), target_[0].CircleRadius());
    }
  }

  for (size_t c = 0; c < calibrator_.NumCameras(); ++c) {
    const Eigen::Matrix3d k_inv = calibrator_.GetCamera(c).camera->K().inverse();

    const CameraAndPose cap = calibrator_.GetCamera(c);
    const Sophus::SE3d t_ck = cap.T_ck;

    // Draw keyframes
    pangolin::glColorBin(c, 2, 0.2);
    for (size_t k = 0; k < calibrator_.NumFrames(); ++k) {
      // Draw the camera frame if we had measurements from it
      std::shared_ptr<VicalibFrame<double> > frame = calibrator_.GetFrame(k);
      if (!frame) continue;

      if (c < frame->has_measurements_from_cam.size() &&
          frame->has_measurements_from_cam[c]) {
        pangolin::glDrawAxis(
            (frame->t_wp_ * t_ck.inverse()).matrix(), 0.01);
      }

      // Draw the IMU frame
      pangolin::glDrawAxis((frame->t_wp_).matrix(), 0.02);

      glColor4f(1, 1, 1, 1);
      // also draw the imu integration for this pose
      if (k < calibrator_.NumFrames() - 1) {
        if (k >= imu_strips_.size()) {
          // also add a strip for the imu integration of this
          // frame to be displayed
          imu_strips_.push_back(std::unique_ptr<GLLineStrip>(new GLLineStrip));
        }
        // now set the points on the strip based on the imu integration
        const std::vector<visual_inertial_calibration::ImuPoseT<double>,
                          Eigen::aligned_allocator<
                            visual_inertial_calibration::ImuPoseT<double> > >
            poses = calibrator_.GetIntegrationPoses(k);
        Vector3dAlignedVec v3d_poses;
        v3d_poses.reserve(poses.size());
        for (const auto& p : poses) {
          v3d_poses.push_back((p.t_wp_).translation());
        }
        imu_strips_[k]->SetPointsFromTrajectory(v3d_poses);
        imu_strips_[k]->Draw();
      }
    }

    // Draw current camera
    if (tracking_good_[c]) {
      if (calibrator_.IsRunning()) {
        t_cw_[c] = calibrator_.GetCamera(c).T_ck *
            calibrator_.GetFrame(calibrator_.NumFrames() - 1)->t_wp_.inverse();
      }
      pangolin::glColorBin(c, 2, 0.5);
      pangolin::glDrawFrustrum(k_inv, width(c), height(c),
                               t_cw_[c].inverse().matrix(),
                               0.05);
    }
  }
}

void VicalibTask::Draw2d() {
  pangolin::View& container = pangolin::Display("vicalib");
  container.ActivateScissorAndClear();
  for (size_t c = 0; c < calibrator_.NumCameras(); ++c) {
    if (container[c].IsShown()) {
      container[c].ActivateScissorAndClear();
      glColor3f(1, 1, 1);

      // Display camera image
      if (!options_->disp_thresh) {
        textures_[c].Upload(image_processing_[c].Img(),
                            GL_LUMINANCE, GL_UNSIGNED_BYTE);
        textures_[c].RenderToViewportFlipY();
      } else {
        textures_[c].Upload(image_processing_[c].ImgThresh(), GL_LUMINANCE,
                            GL_UNSIGNED_BYTE);
        textures_[c].RenderToViewportFlipY();
      }

      // Setup orthographic pixel drawing
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      glOrtho(-0.5, width(c) - 0.5, height(c) - 0.5, -0.5, 0, 1.0);
      glMatrixMode(GL_MODELVIEW);

      const std::vector<calibu::Conic,
                        Eigen::aligned_allocator<calibu::Conic> >& conics =
          conic_finder_[c].Conics();
      if (options_->disp_lines) {
        for (std::list<calibu::LineGroup>::const_iterator it =
                 target_[c].LineGroups().begin();
             it != target_[c].LineGroups().end(); ++it) {
          glColor3f(0.5, 0.5, 0.5);
          glBegin(GL_LINE_STRIP);
          for (std::list<size_t>::const_iterator el = it->ops.begin();
               el != it->ops.end(); ++el) {
            const Eigen::Vector2d p = conics[*el].center;
            glVertex2d(p(0), p(1));
          }
          glEnd();
        }
      }

      if (tracking_good_[c] && options_->disp_barcode) {
        static const int kImageBoundaryRegion = 10;
        const std::vector<Eigen::Vector3d,
            Eigen::aligned_allocator<Eigen::Vector3d> >& code_points =
                target_[c].Code3D();
        const CameraAndPose cap = calibrator_.GetCamera(c);
        const unsigned char* im = image_processing_[c].ImgThresh();
        unsigned char id = 0;
        bool found = true;
        for (size_t k = 0; k < code_points.size(); k++) {
          const Eigen::Vector3d& xwp = code_points[k];
          Eigen::Vector2d pt;
          pt = cap.camera->Project(t_cw_[c] * xwp);
          if (pt[0] < kImageBoundaryRegion ||
              pt[0] >= images_->at(c)->Width() - kImageBoundaryRegion ||
              pt[1] < kImageBoundaryRegion ||
              pt[1] >= images_->at(c)->Height() - kImageBoundaryRegion) {
            found = false;
            continue;
          }

          int idx = image_processing_[c].Width() * round(pt[1]) + round(pt[0]);
          if (im[idx] == 0) {
            glColor3f(0.0, 1.0, 0.0);
            id |= 1 << c;
          } else {
            glColor3f(1.0, 0.0, 0.0);
          }
          pangolin::glDrawRect(pt[0] - 5, pt[1] - 5, pt[0] + 5, pt[1] + 5);
        }
        if (found) {
          LOG(INFO) << "ID: " << id;
        }
      }

      if (options_->disp_cross) {
        for (size_t i = 0; i < conics.size(); ++i) {
          const Eigen::Vector2d pc = conics[i].center;
          pangolin::glColorBin(target_[c].Map()[i].value, 2);
          pangolin::glDrawCross(pc, conics[i].bbox.Width() * 0.75);
        }
      }

      if (options_->disp_bbox) {
        for (size_t i = 0; i < conics.size(); ++i) {
          const Eigen::Vector2i grid_point =
              tracking_good_[c] ?
              target_[c].Map()[i].pg : Eigen::Vector2i(0, 0);

          if (0 <= grid_point[0] && grid_point[0] < grid_size_[0] &&
              0 <= grid_point[1] && grid_point[1] < grid_size_[1]) {
            pangolin::glColorBin(grid_point[1] * grid_size_[0] + grid_point[0],
                                 grid_size_[0] * grid_size_[1]);
            calibu::glDrawRectPerimeter(conics[i].bbox);
          }
        }
      }
    }
  }
}
#endif  // HAVE_PANGOLIN

void VicalibTask::Draw() {
#ifdef HAVE_PANGOLIN
  options_->disp_mse = calibrator_.MeanSquaredError();
  options_->disp_frame = num_frames_;

  Draw2d();
  Draw3d();
#endif  // HAVE_PANGOLIN
}

void VicalibTask::Finish(const std::string& output_filename) {
  calibrator_.Stop();
  calibrator_.PrintResults();
}

std::vector<bool> VicalibTask::AddSuperFrame(
    const std::shared_ptr<hal::ImageArray>& images) {
  images_ = images;

  int num_new_frames = 0;
  std::vector<bool> valid_frames;
  valid_frames.resize(images_->Size());


  DLOG(INFO) << "Frame timestamps: ";
  for (int ii = 0; ii < images_->Size(); ++ii) {
    std::shared_ptr<hal::Image> image = images_->at(ii);

    const double timestamp =
        FLAGS_use_system_time ? images->Ref().system_time() :
        (image->Timestamp() == 0 ? images->Timestamp() : image->Timestamp());

    LOG(INFO) << ii << ": " << std::fixed << " image: " << image->Timestamp() <<
                 " images: " << images->Timestamp() << " sys: " <<
                 images->Ref().system_time() << " final: " << timestamp;

    // If we're finding the time offset, initialize it here.
    if (FLAGS_find_time_offset && FLAGS_calibrate_imu &&
        image_time_offset == -1) {
      if (FLAGS_use_system_time) {
        // Then there is no need to set the image time offset, as timestamps
        // should already be synchronized.
        image_time_offset = 0;
      } else {
        if(calibrator_.imu_buffer().elements_.size() != 0){
          image_time_offset =
              (calibrator_.imu_buffer().elements_[0].time - timestamp);

          LOG(INFO) << "Setting initial time offset to " <<
                       image_time_offset;
        } else {
          LOG(INFO) << "Not added due to missing IMU.";
          valid_frames.assign(images_->Size(), false);
          return valid_frames;
        }
      }
    }

    if (timestamp != frame_times_[ii] || !FLAGS_calibrate_imu) {
      num_new_frames++;
      frame_times_[ii] = timestamp + image_time_offset;
      valid_frames[ii] = true;
    } else {
      valid_frames[ii] = false;
    }
  }

  // This is true if we have at least a single new frame
  bool is_new_frame = num_new_frames > 0;

  if (is_new_frame) {
    current_frame_time_ = frame_times_[0];
    DLOG(INFO) << "Adding frame at time " << current_frame_time_;

    AddImageMeasurements(valid_frames);
  } else if (!is_new_frame) {
    LOG(INFO) << "Frame duplicated";
  }

  num_frames_++;
  return valid_frames;
}

void VicalibTask::AddIMU(const hal::ImuMsg& imu) {
  if (!imu.has_accel()) {
    LOG(ERROR) << "ImuMsg missing accelerometer readings";
  } else if (!imu.has_gyro()) {
    LOG(ERROR) << "ImuMsg missing gyro readings";
  } else {
    Eigen::VectorXd gyro, accel;
    ReadVector(imu.accel(), &accel);
    ReadVector(imu.gyro(), &gyro);
    if (calibrator_.AddImuMeasurements(
          gyro, accel, FLAGS_use_system_time ? imu.system_time() :
                                               imu.device_time())) {
      LOG(INFO) << "TIMESTAMPINFO: IMU Packet device: "
               << std::fixed << imu.device_time() << " sys: " <<
                  imu.system_time() << " a: " << accel.transpose() << " g: " <<
                  gyro.transpose() << std::endl;
    }
  }
}

/**
 * Name: CameraCalibrationsDiffer
 *  @args
 *    const CameraAndPose &last:    This will represent previously guessed or
 *                                  estimated parameteres(i.e. Intrinsic and
 *                                  Extrinsics).
 *    const CameraAndPose &current: This will represent parameters estimated in
 *                                  current run.
 *
 *  @return
 *    bool: Return parameter is true if any of the parameter values for last and
 *    current vary more than specified threshold. If all are close to expected
 *    value than it returns false, which is a successful run.
 **/
bool CameraCalibrationsDiffer(const CameraAndPose& last,
                         const CameraAndPose& current) {
  // First comparing intrinsics of the camera.
  if (last.camera->Type() != current.camera->Type()) {
    LOG(ERROR) << "Camera calibrations are different types: "
               << last.camera->Type() << " vs. "
               << current.camera->Type();
    return true;
  }

  Eigen::VectorXd lastParams = last.camera->GetParams();
  Eigen::VectorXd currentParams = current.camera->GetParams();

  LOG(INFO) << "Comparing old camera calibration: " << lastParams
            << " to new calibration: " << currentParams;

  if (std::abs(lastParams[0] - currentParams[0]) > FLAGS_max_fx_diff) {
    LOG(ERROR) << "fx differs too much ("
               << lastParams[0] - currentParams[0] << ")";
    return true;
  } else if (std::abs(lastParams[1] - currentParams[1]) > FLAGS_max_fy_diff) {
    LOG(ERROR) << "fy differs too much ("
               << lastParams[1] - currentParams[1] << ")";
    return true;
  } else if (std::abs(lastParams[2] - currentParams[2]) > FLAGS_max_cx_diff) {
    LOG(ERROR) << "cx differs too much ("
               << lastParams[2] - currentParams[2] << ")";
    return true;
  } else if (std::abs(lastParams[3] - currentParams[3]) > FLAGS_max_cy_diff) {
    LOG(ERROR) << "cy differs too much ("
               << lastParams[3] - currentParams[3] << ")";
    return true;
  }

  if (current.camera->Type() == "FovCamera" &&
      std::abs(lastParams[4] - currentParams[4]) > FLAGS_max_fov_w_diff) {
    LOG(ERROR) << "fov distortion differs too much ("
               << lastParams[4] - currentParams[4] << ")";
    return true;
  } else if (current.camera->Type() == "Poly3Camera") {
    double d1 = std::abs(lastParams[4] - currentParams[4]);
    double d2 = std::abs(lastParams[5] - currentParams[5]);
    double d3 = std::abs(lastParams[6] - currentParams[6]);

    if (d1 > FLAGS_max_poly3_diff_k1 ||
        d2 > FLAGS_max_poly3_diff_k2 ||
        d3 > FLAGS_max_poly3_diff_k3) {
      LOG(ERROR) << "poly3 distortion differs too much ("
                 << d1 << ", " << d2 << ", " << d3 << ")";
      return true;
    }
  }

  // Now, comparison for extrinsics will start.
  // First, comparison of psoition of camera. Comparison here is done by
  // calculating distance as that requires user to specify only one threshold.
  Eigen::Vector3d lastTrans = last.T_ck.translation();
  Eigen::Vector3d currentTrans = current.T_ck.translation();
  Eigen::Vector3d squareCurrentToLast = (lastTrans - currentTrans);
  double dist = squareCurrentToLast.norm();
  if (dist > FLAGS_max_camera_trans_diff) {
    LOG(ERROR) << "Position of camera differs by " << dist
               << "more than expected ("
               << FLAGS_max_camera_trans_diff << ").";
    return true;
  }

  // Second, comparison for orientation is done.
  // Both inverse of old is multiplied with the new estimate this whould result
  // in and Identity matrix, ideally. Angles along x,y,z are computed with the
  // new rotation matrix and then compared with threshold.
  Sophus::SO3d rot_diff = last.T_ck.so3().inverse() * current.T_ck.so3();
  Eigen::Matrix3d mat_rot_diff = rot_diff.matrix();

  float angle_x = atan2(mat_rot_diff(2, 1), mat_rot_diff(2, 2));
  float root_square_elem = std::sqrt(mat_rot_diff(2, 1) * mat_rot_diff(2, 1) +
                                     mat_rot_diff(2, 2) * mat_rot_diff(2, 2));
  float angle_y = atan2(-1 * mat_rot_diff(2, 0), root_square_elem);
  float angle_z = atan2(mat_rot_diff(1, 0), mat_rot_diff(0, 0));

  if (std::abs(angle_x) > FLAGS_max_camera_angle_diff ||
      std::abs(angle_y) > FLAGS_max_camera_angle_diff ||
      std::abs(angle_z) > FLAGS_max_camera_angle_diff) {
    LOG(ERROR) << "Camera orientations are farther apart than expected"
              << " (" << FLAGS_max_camera_angle_diff << ")."
              << " Difference along x,y,z axis is: "
              << angle_x << ", " << angle_y << ", " << angle_z;
    return true;
  }

  return false;
}

bool IMUCalibrationDiffer(const Vector6d &last,
                          const Vector6d &current) {
  Vector6d diff = last - current;

  if (std::abs(diff(0)) < FLAGS_max_imu_gyro_diff ||
      std::abs(diff(1)) < FLAGS_max_imu_gyro_diff ||
      std::abs(diff(2)) < FLAGS_max_imu_gyro_diff ) {
    LOG(ERROR) << "IMU bias(es) for gyroscope differ ("
              << diff(0) << ", " << diff(1) << ", " << diff(2)
              << " ) more than expected (" << FLAGS_max_imu_gyro_diff << ")";
    return true;
  }

  if (std::abs(diff(3)) < FLAGS_max_imu_accel_diff ||
      std::abs(diff(4)) < FLAGS_max_imu_accel_diff ||
      std::abs(diff(5)) < FLAGS_max_imu_accel_diff ) {
    LOG(ERROR) << "IMU bias(es) for accelrometer differ ("
              << diff(3) << ", " << diff(4) << ", " << diff(5)
              << " ) more than expected (" << FLAGS_max_imu_accel_diff << ")";
    return true;
  }
  return false;
}

bool VicalibTask::IsSuccessful() const {
  std::vector<double> errors = calibrator_.GetCameraProjRMSE();
  for (size_t ii = 0; ii < nstreams_; ++ii) {
    if (errors[ii] > max_reproj_errors_[ii]) {
      LOG(WARNING) << "Reprojection error of " << errors[ii]
                   << " was greater than maximum of "
                   << max_reproj_errors_[ii]
                   << " for camera " << ii;
      return false;
    }
  }

  // Check that camera parameters are within thresholds of previous
  // run if we initialized our optimization with it
  if (FLAGS_has_initial_guess) {
    for (size_t i = 0; i < input_cameras_.size(); ++i) {
      if (CameraCalibrationsDiffer(input_cameras_[i],
                                   calibrator_.GetCamera(i))) {
        return false;
      }
    }
    if (IMUCalibrationDiffer(input_imu_biases_, calibrator_.GetBiases()))
      return false;
  }
  return true;
}
}  // namespace visual_inertial_calibration
