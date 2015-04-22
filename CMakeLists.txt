cmake_minimum_required( VERSION 2.8 )
project("VICALIB")

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH
endif()


set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/)
include( install_package )
include( def_executable )

set(VICALIB_VERSION_MAJOR 0)
set(VICALIB_VERSION_MINOR 1)
set(VICALIB_VERSION ${VICALIB_VERSION_MAJOR}.${VICALIB_VERSION_MINOR})

mark_as_advanced( EXPORT_VICALIB ) # make it a bit harder to change this option.
option(EXPORT_VICALIB "Should VICALIB be exported." OFF)

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Build type not set (defaults to release)")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

option(BUILD_SHARED_LIBS "Build Shared Library" ON)

set( CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}" )
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

if (NOT ANDROID)
  add_definitions(-DBUILD_GUI)
else()
  add_definitions(-UBUILD_GUI)
endif()

################################################################################
# Find required dependencies

find_package(Calibu CONFIG)
find_package(Ceres)
find_package(CVars)
find_package(GFlags)
find_package(GLog)
find_package(HAL)
find_package(OpenCV2)
find_package(Pangolin)

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
  ${CVars_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${OpenCV2_LIBRARIES}
  dl)

list(APPEND PROJ_INCLUDE_DIRS
  ${Pangolin_INCLUDE_DIRS}
  ${HAL_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${CVars_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  )


# Basic includes / libraries
list( APPEND LIB_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include"
                         "${CMAKE_CURRENT_SOURCE_DIR}"
                          ${EIGEN_INCLUDE_DIR}
                          ${Sophus_INCLUDE_DIR}
                          ${SceneGraph_INCLUDE_DIRS})


set(INC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/include/vicalib)
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

set(SRC_PREFIX src)
set(VICALIB_SRCS
    ${SRC_PREFIX}/vicalib-engine.cc
    ${SRC_PREFIX}/vicalib-task.cc
)

set(VICALIB_CONDITIONS
    Calibu_FOUND
  Ceres_FOUND
  GFLAGS_FOUND
  GLOG_FOUND
  HAL_FOUND
  OpenCV2_FOUND
  Pangolin_FOUND
  )

list( APPEND ${PROJ_INCLUDE_DIRS} ${LIB_INC_DIR} )

include_directories( ${USER_INC}
    ${PROJ_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/include
    )

add_library( vicalib ${VICALIB_HEADERS} ${VICALIB_SRCS} )
target_link_libraries( vicalib ${LINK_LIBS} )

# Create the CANDLEConfig.cmake file for the build tree.
set(EXPORT_VICALIB_INC
  ${PROJ_INCLUDE_DIRS}
  )

install_package(
    PKG_NAME VICALIB
  LIB_NAME vicalib
  VERSION ${VICALIB_VERSION}
  DESCRIPTION "Visual-inertial calibration routine."
  INSTALL_INCLUDE_DIR true
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/vicalib
  INCLUDE_DIRS ${EXPORT_VICALIB_INC}
  LINK_LIBS ${LINK_LIBS}
  )

# For the examples to find this and only this version of VICALIB
set( VICALIB_DIR ${CMAKE_CURRENT_BINARY_DIR} )
set( VICALIB_INCLUDE_DIRS ${USER_INC} )
set( VICALIB_LIBRARIES ${LINK_LIBS} vicalib )
link_libraries( ${VICALIB_LIBRARIES} vicalib )

option(BUILD_TESTS "Build tests." ON)
if( BUILD_TESTS )
  enable_testing()
  add_subdirectory( testing )
endif()

option(BUILD_APPLICATIONS "Build applications." ON)
if( BUILD_APPLICATIONS )
  add_subdirectory( applications )
endif()

###########################
 include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)

 add_custom_target(uninstall
     COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
###########################
