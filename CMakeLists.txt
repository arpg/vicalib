cmake_minimum_required(VERSION 2.8)
project(visual_inertial_calibration)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

include(def_executable)

if (ANDROID)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mimplicit-it=thumb")
endif()

if (NOT ANDROID)
  add_definitions(-DBUILD_GUI)
else()
  add_definitions(-UBUILD_GUI)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall -Wextra -Wno-unused-parameter -Wno-deprecated")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(Calibu)
find_package(Ceres)
find_package(CVARS)
find_package(GFlags)
find_package(GLog)
find_package(HAL)
find_package(OpenCV2)
find_package(Pangolin)
find_package(PbMsgs)
find_package(MINIGLOG)

include_directories(
  ${Pangolin_INCLUDE_DIRS}
  ${HAL_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${CVARS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${MINIGLOG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(VICALIB_HDRS
  include/visual-inertial-calibration/boxcar-filter.h
  include/visual-inertial-calibration/calibration-stats.h
  include/visual-inertial-calibration/ceres-cost-functions.h
  include/visual-inertial-calibration/gl-line-strip.h
  include/visual-inertial-calibration/interpolation-buffer.h
  include/visual-inertial-calibration/local-param-se3.h
  include/visual-inertial-calibration/types.h
  include/visual-inertial-calibration/vicalib-task.h
  include/visual-inertial-calibration/vicalibrator-utils.h
  include/visual-inertial-calibration/vicalibrator.h
  )
set(VICALIB_SRCS
  src/main.cc
  src/vicalib-engine.cc
  src/vicalib-task.cc
  )

set(VICALIB_CONDITIONS
  Calibu_FOUND
  Ceres_FOUND
  GFLAGS_FOUND
  GLOG_FOUND
  HAL_FOUND
  OpenCV2_FOUND
  Pangolin_FOUND
  MINIGLOG_FOUND
  )

if(NOT ANDROID)
  def_executable(vicalib
    SOURCES ${VICALIB_SRCS} ${VICALIB_HDRS}
    DEPENDS hal pangolin calibu pbmsgs miniglog
    LINK_LIBS
    ${CERES_LIBRARIES}
    ${CVARS_LIBRARY}
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${OpenCV2_LIBRARIES}
    ${PbMsgs_LIBRARIES}
    ${MINIGLOG_LIBRARIES}
    dl
    CONDITIONS ${VICALIB_CONDITIONS}
    )
endif()
