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
find_package( Calibu REQUIRED )
find_package( Ceres REQUIRED )
find_package( CVars REQUIRED )
find_package( GFlags REQUIRED )
find_package( GLog REQUIRED )
find_package( HAL REQUIRED )
find_package( OpenCV REQUIRED COMPONENTS core)
if(OpenCV_VERSION_MAJOR EQUAL 2)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
elseif(OpenCV_VERSION_MAJOR EQUAL 3)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
endif()
find_package( Pangolin REQUIRED )

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
  ${GFlags_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  dl)

list(APPEND PROJ_INCLUDE_DIRS
  ${Pangolin_INCLUDE_DIRS}
  ${HAL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Calibu_INCLUDE_DIRS}
  ${CVars_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GFlags_INCLUDE_DIR}
  )


# Basic includes / libraries
list( APPEND LIB_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
                          ${EIGEN_INCLUDE_DIR}
                          ${Sophus_INCLUDE_DIR}
                          ${SceneGraph_INCLUDE_DIRS})

set(SRC_PREFIX src)
set(VICALIB_SRCS
    ${SRC_PREFIX}/vicalib-engine.cc
    ${SRC_PREFIX}/main.cc
    ${SRC_PREFIX}/vicalib-task.cc
)

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

include_directories( ${USER_INC}
    ${PROJ_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/include
    )

add_executable( vicalib ${VICALIB_SRCS} )
target_link_libraries( vicalib ${LINK_LIBS} )

add_executable( tracker src/tracker.cc )
target_link_libraries( tracker ${LINK_LIBS} )

install(TARGETS vicalib RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(TARGETS tracker RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)
add_custom_target(uninstall
     COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
