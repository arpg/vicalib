vicalib
=======

Vicalib is a calibration tool for visual or visual-inertial sensor systems. The system is able to calibrate camera intrinsics for pinhole, polynomial and r-theta camera models, as well as the camera-to-imu extrinsics, imu biases and scale factors for visual inertial system. Dependencies are as follows:

Dependencies
============

- Calibu (https://github.com/arpg/calibu): Camera calibration library. The dependency is used for target generation, detection and tracking.
- HAL (https://github.com/arpg/hal): Hardware Abstraction Layer. The dependency includes drivers to read camera and IMU information from files or devices.
- Pangolin (https://github.com/arpg/Pangolin) forked from (https://github.com/stevenlovegrove/Pangolin): OpenGL GUI and utility library
- Ceres Solver (http://ceres-solver.org/): Non-linear solver used to estimate the calibration parameters
- CVARS (https://github.com/arpg/GLConsole): GUI interface to variables to control program operation
- GFlags (https://code.google.com/p/gflags/): Google command line flag library
- GLog (https://code.google.com/p/google-glog/): Google logging library
- OpenCV (http://opencv.org/): Computer vision utility library

Building
========
Make sure all dependencies are either built with CMake or installed (so that CMake can find relevant paths). Use CMake to generate makefiles and run make.

Usage
=====
Vicalib accepts images and IMU data passed via command line options with the following format 

vicalib [-models <cam_models>] [-cam <camera_uri>] [-imu <imu_uri] [-options]

-models defines the specific camera model to use for intrinsics calibration. These camera models are defined as follows:

  "fov": FOV camera model. suitable for wide angle (fish-eye) lenses [Straight lines have to be straight, F Devernay, O Faugeras, Machine Vision and Applications 13 (1), 14-24]
  
  "poly2": Polynomial camera model with two distortion parameters.
  
  "poly3": Polynomial camera model with three distortion parameters.
  "kb4": The Kannala Brandt camera model
  
  As an example, the following argument would signal that a stereo pair of FOV camera models should be calibrated:
  -model fov,fov
  
-cam specifies the URI for the camera information. The URI format is as follows

  -cam driver:[<driver_options>]//<driver_path>
  
  There are many camera drivers available in HAL (which is a dependency). The drivers are located in /Hal/Camera/Drivers. The specific usage for a library is provided in the <driver_name>Factory.cpp file. For example the FileReader driver is used to read images from disk. The FileReaderFactory.cpp file contains a list of <driver_options> available. A sample URI for the FileReader driver is as follows 
  file:[loop=1,startframe=100]///usr/local/datasets/my_dataset/[left,right]*pgm
  Which will attempt to read files with the [left,right]*pgm regex from the directory /usr/local/datasets/my_dataset/, skipping the first 100 frames and looping back to the first frame when reaching the end of the files.
  (Note that the FileReader driver has the capability to extract timestamps from file names, in order to sychronize visual and inertial information).
  
-imu specifies the URI for IMU information. It is formatted as follows

  -imu driver:[<driver_options>]//<driver_path>
  
  Similar to camer drivers, the IMU drivers are from the HAL library and are available from /HAL/IMU/Drivers. The specific usage can be found in the <driver_name>Factory.cpp file. For example the CsvDriver accepets a directory as the <driver_path> directory and expects the accel.txt, gyro.txt, mag.txt and timestamp.txt files in the given directory. These files must be CSV files containing the relevant sensor information as per CsvDriver.cpp.
  
-options specifies a number of options to control the optimization. A description of these options can be obtained by running

  vicalib --help
  
  Note: If any of the boolean options have a default value of true, they can be turned off by prefixing them with "no" for example the -calibrate_imu flag has a default value of true. Passing -nocalibrate_imu turns this flag off.
  
  The description for a number of important options is as follows:
  -calibrate_imu: whether or not to including IMU information in the calibration. Will include by default the IMU to camera extrinsics, biases and scale factors
  -calibrate_intrinsics: calibrate the camera intrinsics as well as the extrinsics. (useful if calibrating a stereo rig with known intrinsics)
  -frame_skip: number of frames to skip before adding a measurement frame. (useful if there are too many frames in the dataset and you wish to skip some frames).
  -grid_preset: which grid preset to use. 0 indicates the small grid template, and 1 indicates the large template.
  
  -has_initial_guess: if a cameras.xml file already exists, whether to use the values within the file to initialize the optimization. Will speed up convergence as some initialization steps will be skipepd/.
  
  -num_vicalib_frames: Will begin the calibration after n frames. Useful if the dataset is too long and only a certain number of frames should be used.
  
  -use_only_when_static: only use frames where the IMU is not moving (thresholds set by -static_accel_threshold, -static_gyro_threshold and -static_threshold_preset).
  
  -find_time_offset: whether to optimize the time difference between the IMU and camera
  
  -use_system_time: whether or not to use the system_time or device_time timestamps. 
  

