cmake_minimum_required( VERSION 2.8 )
include(def_library)

################################################################################
# Find required dependencies

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
find_package(Protobuf)

# Prefer OSX_MODIFIED_GLUT > FREEGLUT > GLUT
if(FREEGLUT_FOUND AND NOT HAVE_MODIFIED_OSXGLUT)
  set(HAVE_FREEGLUT 1)
  list(APPEND LIB_INC_DIR  ${FREEGLUT_INCLUDE_DIR} )
  list(APPEND LINK_LIBS ${FREEGLUT_LIBRARY} )
elseif(GLUT_FOUND)
  list(APPEND LIB_INC_DIR  ${GLUT_INCLUDE_DIR} )
  list(APPEND LINK_LIBS ${GLUT_LIBRARY} )
endif()

list(APPEND LINK_LIBS
  ${HAL_LIBRARIES}
  ${Calibu_LIBRARIES}
  ${CERES_LIBRARIES}
  ${CVARS_LIBRARY}
  ${PROTOBUF_LIBRARY}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${OpenCV2_LIBRARIES}
  ${PbMsgs_LIBRARIES}
  ${MINIGLOG_LIBRARIES}
  dl)

list(APPEND PROJ_INCLUDE_DIRS
  ${Pangolin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${HAL_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${CVARS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${MINIGLOG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)


# Basic includes / libraries
list( APPEND LIB_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include"
                         "${CMAKE_CURRENT_SOURCE_DIR}/.."
                          ${EIGEN_INCLUDE_DIR}
                          ${Sophus_INCLUDE_DIR}
                          ${SceneGraph_INCLUDE_DIRS})


set(INC_PREFIX ${CMAKE_SOURCE_DIR}/include/vicalib)
set(VICALIB_HDRS
    ${INC_PREFIX}/boxcar-filter.h
    ${INC_PREFIX}/calibration-stats.h
    ${INC_PREFIX}/ceres-cost-functions.h
    ${INC_PREFIX}/gl-line-strip.h
    ${INC_PREFIX}/interpolation-buffer.h
    ${INC_PREFIX}/local-param-se3.h
    ${INC_PREFIX}/types.h
    ${INC_PREFIX}/vicalib-task.h
    ${INC_PREFIX}/vicalibrator-utils.h
    ${INC_PREFIX}/vicalibrator.h
    )

set(VICALIB_SRCS
  vicalib-engine.cc
  vicalib-task.cc
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

list( APPEND ${PROJ_INCLUDE_DIRS} ${LIB_INC_DIR} )

include_directories(${PROJ_INCLUDE_DIRS})

include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )

def_library(${LIBRARY_NAME}
  SOURCES ${VICALIB_HDRS} ${VICALIB_SRCS}
  DEPENDS hal pangolin calibu pbmsgs miniglog
  LINK_LIBS ${LINK_LIBS}
  PACKAGE ${PROJECT_NAME}
  CONDITIONS ${VICALIB_CONDITIONS}
  )

# Create the CANDLEConfig.cmake file for the build tree.
set(EXPORT_VICALIB_INC
  ${PROJ_INCLUDE_DIRS}
  )

export_package(${PROJECT_NAME}
  TARGETS ${LIBRARY_NAME}
  LIBRARY ${LIBRARY_NAME}
  LIBRARIES ${LIBRARY_NAME}
  INCLUDE_DIRS ${EXPORT_VICALIB_INC}
  )

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/VICALIBConfig.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${LIBRARY_NAME}
  )

# For the examples to find this and only this version of VICALIB
set(VICALIB_DIR ${CMAKE_CURRENT_BINARY_DIR})


