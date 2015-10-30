#include <gflags/gflags.h>
#include <glog/logging.h>
#include <HAL/Camera/CameraDevice.h>
#include <pangolin/pangolin.h>
#include <calibu/target/TargetGridDot.h>
#include <calibu/pose/Tracker.h>
#include <calibu/pose/Pnp.h>
#include <calibu/cam/camera_xml.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

template <template<typename ...> class ContainerT, typename T, typename ...Args>
using aligned = ContainerT<T, Args..., Eigen::aligned_allocator<T> >;

template <typename T>
using aligned_vector = aligned<std::vector, T>;



static const char* g_usage =
"Examples: \n\n"
" tracker -grid_preset=letter -cam file:[file=cameras.xml]//*.pgm\n\n";

DEFINE_string(cam, "file://*.pgm", 
    "\nCamera URI.  Defaults to file://*.pgm which will will use all\n"
    "the .pgm files in the current directory.\n");

DEFINE_string(target, "letter",
    "\nWhich target preset to use.  Use \"small\" for small GWU grid, \"large\""
    "for large Google grid, \"letter\" for the US paper sized grid (default).\n");

DEFINE_string(model, "cameras.xml", "\nCamera model file to load.\n" );


int main(int argc, char *argv[]) 
{
  if( argc == 1 ){
    google::SetUsageMessage(g_usage);
    google::ShowUsageWithFlags(argv[0]);
    return -1;
  }
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Setup the GUI
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("Tracker",640,480);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
      pangolin::ModelViewLookAt(-1,1,-1, 0,0,0, pangolin::AxisY)
      );

  // Aspect ratio allows us to constrain width and height whilst fitting within specified
  // bounds. A positive aspect ratio makes a view 'shrink to fit' (introducing empty bars),
  // whilst a negative ratio makes the view 'grow to fit' (cropping the view).
  pangolin::View& d_cam = pangolin::Display("cam");
  d_cam.SetBounds(0,1.0f,0,1.0f,-640/480.0);
  d_cam.SetHandler(new pangolin::Handler3D(s_cam));

  // This view will take up no more than a third of the windows width or height, and it
  // will have a fixed aspect ratio to match the image that it will display. When fitting
  // within the specified bounds, push to the top-left (as specified by SetLock).
  pangolin::View& d_image = pangolin::Display("image");
  d_image.SetBounds(2/3.0f,1.0f,0,1/3.0f,640.0/480);
  d_image.SetLock(pangolin::LockLeft, pangolin::LockTop);

  // parse uri, init camera stream
  hal::Camera camera = hal::Camera( hal::Uri(FLAGS_cam) );
  std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
  const int width =  camera.Width();
  const int height = camera.Height();
  pangolin::GlTexture imageTexture(width,height,GL_RGB,false,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);

  calibu::TargetGridDot target( FLAGS_target ); // load a target dot grid from a preset
  calibu::Tracker tracker( target, width, height );

  std::shared_ptr<calibu::Rigd> rig = calibu::ReadXmlRig( FLAGS_model );
  std::shared_ptr<calibu::CameraInterfaced> model = rig->cameras_[0];

  Eigen::Matrix3d Kinv = model->K().inverse();

  std::deque<Eigen::Matrix4d> poses;
  // Default hooks for exiting (Esc) and fullscreen (tab).
  while(!pangolin::ShouldQuit()) {
    std::shared_ptr<hal::Image> im;
    if( camera.Capture(*images) ){
      im = images->at(0);
      if( im->Width() != width ){
        printf("error camera width and image width do not match\n");
        return -1;
      }
      // track the target
      int pitch = width;
      if( !tracker.ProcessFrame( model, im->data(), im->Width(), im->Height(), pitch ) ){
        printf("failed to track!\n");
        continue;
      }
      std::cout << tracker.PoseT_gw().matrix() << std::endl;
      poses.push_back( tracker.PoseT_gw().matrix() );
      imageTexture.Upload( im->data(),GL_LUMINANCE,GL_UNSIGNED_BYTE );
      if( poses.size() > 1500 ){
        poses.pop_front();
      } 
    }


    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    d_image.Activate();
    imageTexture.RenderToViewportFlipY();

    d_cam.Activate(s_cam);
    for( size_t ii = 0; ii < poses.size(); ii++ ){
      pangolin::glDrawAxis( poses[ii], 0.01 );
    }

    // Draw current camera
    glColor3f(1.0,1.0,1.0);
    pangolin::glDrawFrustrum(Kinv,width,height,0.05);

    pangolin::FinishFrame();
  }

  return 0;
}

