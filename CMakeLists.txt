cmake_minimum_required( VERSION 2.8 )
project("vicalib")

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
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" AND NOT ANDROID)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

option(BUILD_GUI "Build GUI" ON)

if (NOT ANDROID AND BUILD_GUI)
  add_definitions(-DBUILD_GUI)
else()
  add_definitions(-UBUILD_GUI)
endif()

################################################################################
# Find required dependencies
FIND_PACKAGE( tinyxml2 REQUIRED CONFIG)
find_package( Calibu REQUIRED )
find_package( Ceres REQUIRED )
find_package( gflags REQUIRED CONFIG)
find_package( glog REQUIRED CONFIG)
find_package( protobuf REQUIRED CONFIG)
find_package( hal REQUIRED CONFIG)
find_package( OpenCV REQUIRED COMPONENTS core)
if(OpenCV_VERSION_MAJOR EQUAL 2)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
elseif(OpenCV_VERSION_MAJOR EQUAL 3)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
endif()
if(NOT ANDROID AND BUILD_GUI)
  find_package( Pangolin REQUIRED )
endif()

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
  hal::hal
  ${Calibu_LIBRARIES}
  ${CERES_LIBRARIES}
  gflags_shared
  glog::glog
  ${OpenCV_LIBS}
  dl)

if(NOT ANDROID AND BUILD_GUI)
  list(APPEND LINK_LIBS
      ${Pangolin_LIBRARIES}
    )
endif()

list(APPEND PROJ_INCLUDE_DIRS
  ${Calibu_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${rapidjson_INCLUDE_DIR}
  )

if(NOT ANDROID AND BUILD_GUI)
  list(APPEND LINK_LIBS
      ${Pangolin_INCLUDE_DIRS}
    )
endif()


# Basic includes / libraries
list( APPEND LIB_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
                          ${EIGEN_INCLUDE_DIR}
                          ${Sophus_INCLUDE_DIR}
                          ${SceneGraph_INCLUDE_DIRS})

set(SRC_PREFIX src)
set(VICALIB_SRCS
    ${SRC_PREFIX}/vicalib-engine.cc
    ${SRC_PREFIX}/vicalib-task.cc
)

if(NOT ANDROID)
  list(APPEND VICALIB_SRCS ${SRC_PREFIX}/main.cc)
else()
  list(APPEND VICALIB_SRCS ${SRC_PREFIX}/libmain.cc)
endif()

set(VICALIB_CONDITIONS
  Calibu_FOUND
  Ceres_FOUND
  GFLAGS_FOUND
  GLOG_FOUND
  HAL_FOUND
  OpenCV_FOUND
  Pangolin_FOUND
  )

list( APPEND ${PROJ_INCLUDE_DIRS} ${LIB_INC_DIR} )

link_directories(${HAL_LIBRARY_DIRS})

include_directories( ${USER_INC}
    ${PROJ_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/include
    )

link_directories(
  ${Calibu_LIBRARY_DIRS}
  ${HAL_LIBRARY_DIRS}
  ${CVars_LIBRARY_DIRS}
  ${Pangolin_LIBRARY_DIRS}
)

IF(WIN32)
    set_property(SOURCE ${SRC_PREFIX}/vicalib-task.cc APPEND_STRING PROPERTY COMPILE_FLAGS " /bigobj ")
ENDIF()
if(NOT ANDROID)
  add_executable( vicalib ${VICALIB_SRCS} )
else()
  add_library( vicalib SHARED ${VICALIB_SRCS} )
  set_target_properties(vicalib PROPERTIES LINK_FLAGS " -Wl,--build-id ")
endif()
target_link_libraries( vicalib ${LINK_LIBS} )


target_include_directories (vicalib BEFORE PUBLIC
  "$<INSTALL_INTERFACE:include>"
  )

#add_executable( tracker src/tracker.cc )
#target_link_libraries( tracker ${LINK_LIBS} )

include (CMakePackageConfigHelpers)

install(TARGETS vicalib
  EXPORT vicalib-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  PUBLIC_HEADER DESTINATION include/vicalib
  )
#install(TARGETS tracker RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)


set(LOAD_DEPENDENCY "#find_dependency( protobuf )
#find_dependency( gflags )
find_dependency( glog )
#find_dependency( tinyxml2 )
find_dependency( hal )
find_dependency( OpenCV )
find_dependency( Calibu )
find_dependency( Ceres )")

configure_package_config_file (vicalib-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/vicalib-config.cmake
  INSTALL_DESTINATION lib/cmake/vicalib
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file (vicalib-config-version.cmake VERSION
  ${VICALIB_VERSION} COMPATIBILITY SameMajorVersion)

export (TARGETS vicalib NAMESPACE vicalib:: FILE vicalib-targets.cmake)
export (PACKAGE vicalib)

install (FILES
  ${CMAKE_CURRENT_BINARY_DIR}/vicalib-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/vicalib-config-version.cmake
  DESTINATION lib/cmake/vicalib)

install (EXPORT vicalib-targets NAMESPACE vicalib:: DESTINATION lib/cmake/vicalib)


install( FILES ${CMAKE_SOURCE_DIR}/include/vicalib/vicalib_interface.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/vicalib )

if(UNIX)
    install (PROGRAMS scripts/run_imu_calibration.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()
install (PROGRAMS scripts/run_vicalib.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (PROGRAMS scripts/run_benchmark.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (PROGRAMS scripts/run_vicalib_evaluation.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install (FILES data/t26x-initialization.json DESTINATION ${CMAKE_INSTALL_PREFIX}/data)

include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)
add_custom_target(uninstall
     COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
