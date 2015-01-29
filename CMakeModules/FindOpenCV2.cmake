# To find OpenCV 2 library visit http://opencv.willowgarage.com/wiki/
#
# The follwoing variables are optionally searched for defaults
#  OpenCV2_ROOT_DIR:                   Base directory of OpenCV 2 tree to use.
#
# The following are set after configuration is done: 
#  OpenCV2_FOUND
#  OpenCV2_INCLUDE_DIRS
#  OpenCV2_LIBRARIES
#
# $Id: $
#  
# Balazs [2011-01-18]:
# - Created from scratch for the reorganized OpenCV 2 structure introduced at version 2.2
# Jbohren [2011-06-10]:
# - Added OpenCV_ROOT_DIR for UNIX platforms & additional opencv include dir
#
# This file should be removed when CMake will provide an equivalent

#MESSAGE(STATUS "Looking for OpenCV2...")

# typical root dirs of installations, exactly one of them is used
IF(WIN32)
    SET(OpenCV2_POSSIBLE_ROOT_DIRS
        "$ENV{OpenCV_ROOT_DIR}"
        "$ENV{SystemDrive}/OpenCV2.3"                     # Windows: OpenCV 2.3 default installation dir (expected future revision)
        "$ENV{SystemDrive}/OpenCV2.2"                     # Windows: OpenCV 2.2 default installation dir
        "$ENV{SystemDrive}/Program Files/OpenCV2.3"       # 32 bit ProgramFiles dir on Win32;  64 bit ProgramFiles dir on Win64 (expected future revision)
        "$ENV{SystemDrive}/Program Files/OpenCV2.2"       # 32 bit ProgramFiles dir on Win32;  64 bit ProgramFiles dir on Win64
        "$ENV{SystemDrive}/Program Files (x86)/OpenCV2.3" # 32 bit ProgramFiles dir on Win64 (expected future revision)
        "$ENV{SystemDrive}/Program Files (x86)/OpenCV2.2" # 32 bit ProgramFiles dir on Win64
        )
ELSE(WIN32)
    SET(OpenCV2_POSSIBLE_ROOT_DIRS
        "$ENV{ROS_ROOT}/../vision_opencv/opencv2/opencv"
	"$ENV{OpenCV_ROOT_DIR}"                         # *NIX: custom install location (like ROS)
        /usr/local                                      # Linux: default dir by CMake
        /usr                                            # Linux
        /opt/local                                      # OS X: default MacPorts location
	)
ENDIF(WIN32)

# select exactly ONE OpenCV 2 base directory
# to avoid mixing different version headers and libs
FIND_PATH(OpenCV2_ROOT_INC_DIR
          NAMES opencv2/opencv.hpp
          PATHS ${OpenCV2_POSSIBLE_ROOT_DIRS}
          )

# Get parent of OpenCV2_ROOT_INC_DIR. We do this as it is more
# reliable than finding include/opencv2/opencv.hpp directly.
GET_FILENAME_COMPONENT(OpenCV2_ROOT_DIR ${OpenCV2_ROOT_INC_DIR} PATH)

FIND_PATH(OpenCV2_CORE_INCLUDE_DIR
          NAMES core_c.h core.hpp wimage.hpp eigen.hpp internal.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/core")
FIND_PATH(OpenCV2_IMGPROC_INCLUDE_DIR
          NAMES imgproc_c.h imgproc.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/imgproc")
FIND_PATH(OpenCV2_FEATURES2D_INCLUDE_DIR
          NAMES features2d.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/features2d")
FIND_PATH(OpenCV2_NONFREE_INCLUDE_DIR
          NAMES features2d.hpp gpu.hpp nonfree.hpp ocl.hpp 
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/nonfree")
FIND_PATH(OpenCV2_FLANN_INCLUDE_DIR
          NAMES flann.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/flann")
FIND_PATH(OpenCV2_CALIB3D_INCLUDE_DIR
          NAMES calib3d.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/calib3d")
FIND_PATH(OpenCV2_OBJDETECT_INCLUDE_DIR
          NAMES objdetect.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/objdetect")
FIND_PATH(OpenCV2_LEGACY_INCLUDE_DIR
          NAMES compat.hpp legacy.hpp blobtrack.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/legacy")
FIND_PATH(OpenCV2_CONTRIB_INCLUDE_DIR
          NAMES contrib.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/contrib")
FIND_PATH(OpenCV2_HIGHGUI_INCLUDE_DIR
          NAMES   highgui_c.h highgui.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/highgui")
FIND_PATH(OpenCV2_ML_INCLUDE_DIR
          NAMES ml.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/ml")
FIND_PATH(OpenCV2_VIDEO_INCLUDE_DIR
          NAMES tracking.hpp background_segm.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/video")
FIND_PATH(OpenCV2_GPU_INCLUDE_DIR
          NAMES gpu.hpp
          PATHS "${OpenCV2_ROOT_DIR}/include/opencv2/gpu")


# absolute path to all libraries 
SET(OPENCV2_LIBRARY_SEARCH_PATHS "${OpenCV2_ROOT_DIR}/lib")

IF(WIN32)
    SET(OPENCV2_LIBRARY_SEARCH_PATHS
        ${OPENCV2_LIBRARY_SEARCH_PATHS}
        "${OpenCV2_ROOT_DIR}/lib/Release"
        "${OpenCV2_ROOT_DIR}/lib/Debug"
        )
    FIND_LIBRARY(OpenCV2_CORE_LIBRARY
                 NAMES opencv_core230 opencv_core220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_IMGPROC_LIBRARY
                 NAMES opencv_imgproc230 opencv_imgproc220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_FEATURES2D_LIBRARY
                 NAMES opencv_features2d230 opencv_features2d220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_NONFREE_LIBRARY
                 NAMES opencv_nonfree230 opencv_nonfree220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_FLANN_LIBRARY
                 NAMES opencv_flann230 opencv_flann220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_CALIB3D_LIBRARY
                 NAMES opencv_calib3d230 opencv_calib3d220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_OBJDETECT_LIBRARY
                 NAMES opencv_objdetect230 opencv_objdetect220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_LEGACY_LIBRARY
                 NAMES opencv_legacy230 opencv_legacy220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_CONTRIB_LIBRARY
                 NAMES opencv_contrib230 opencv_contrib220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_HIGHGUI_LIBRARY
                 NAMES opencv_highgui230 opencv_highgui220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_ML_LIBRARY
                 NAMES opencv_ml230 opencv_ml220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_VIDEO_LIBRARY
                 NAMES opencv_video230 opencv_video220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_GPU_LIBRARY
                 NAMES opencv_gpu230 opencv_gpu220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_FFMPEG_LIBRARY
                 NAMES opencv_ffmpeg230 opencv_ffmpeg220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_TS_LIBRARY
                 NAMES opencv_ts230 opencv_ts220
                 PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
ELSE(WIN32)
    FIND_LIBRARY(OpenCV2_CORE_LIBRARY       NAMES opencv_core       PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_IMGPROC_LIBRARY    NAMES opencv_imgproc    PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_FEATURES2D_LIBRARY NAMES opencv_features2d PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_NONFREE_LIBRARY    NAMES opencv_nonfree    PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_FLANN_LIBRARY      NAMES opencv_flann      PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_CALIB3D_LIBRARY    NAMES opencv_calib3d    PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_OBJDETECT_LIBRARY  NAMES opencv_objdetect  PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_LEGACY_LIBRARY     NAMES opencv_legacy     PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_CONTRIB_LIBRARY    NAMES opencv_contrib    PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_HIGHGUI_LIBRARY    NAMES opencv_highgui    PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_ML_LIBRARY         NAMES opencv_ml         PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_VIDEO_LIBRARY      NAMES opencv_video      PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    FIND_LIBRARY(OpenCV2_GPU_LIBRARY        NAMES opencv_gpu        PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
ENDIF(WIN32)

IF(ANDROID)
    FIND_LIBRARY(OpenCV2_JAVA_LIBRARY       NAMES opencv_java       PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
ENDIF(ANDROID)

SET(OpenCV2_INCLUDE_DIRS
    ${OpenCV2_ROOT_DIR}/include
    ${OpenCV2_ROOT_DIR}/include/opencv2
    ${OpenCV2_CORE_INCLUDE_DIR}
    ${OpenCV2_IMGPROC_INCLUDE_DIR}
    ${OpenCV2_FEATURES2D_INCLUDE_DIR}
    #${OpenCV2_NONFREE_INCLUDE_DIR}
    ${OpenCV2_FLANN_INCLUDE_DIR}
    ${OpenCV2_CALIB3D_INCLUDE_DIR}
    ${OpenCV2_OBJDETECT_INCLUDE_DIR}
    ${OpenCV2_LEGACY_INCLUDE_DIR}
    ${OpenCV2_CONTRIB_INCLUDE_DIR}
    ${OpenCV2_HIGHGUI_INCLUDE_DIR}
    ${OpenCV2_ML_INCLUDE_DIR}
    ${OpenCV2_VIDEO_INCLUDE_DIR}
    )

IF(NOT ANDROID)
    LIST(APPEND OpenCV2_INCLUDE_DIRS ${OpenCV2_GPU_INCLUDE_DIR})
ENDIF()

SET(OpenCV2_LIBRARIES
    ${OpenCV2_CORE_LIBRARY}
    ${OpenCV2_IMGPROC_LIBRARY}
    ${OpenCV2_FEATURES2D_LIBRARY}
    #${OpenCV2_NONFREE_LIBRARY}
    ${OpenCV2_FLANN_LIBRARY}
    ${OpenCV2_CALIB3D_LIBRARY}
    ${OpenCV2_OBJDETECT_LIBRARY}
    ${OpenCV2_LEGACY_LIBRARY}
    ${OpenCV2_CONTRIB_LIBRARY}
    ${OpenCV2_HIGHGUI_LIBRARY}
    ${OpenCV2_ML_LIBRARY}
    ${OpenCV2_VIDEO_LIBRARY}
    )

IF(NOT ANDROID)
    LIST(APPEND OpenCV2_LIBRARIES ${OpenCV2_GPU_LIBRARY})
else()
    LIST(APPEND OpenCV2_LIBRARIES ${OpenCV2_JAVA_LIBRARY})
ENDIF()


IF(WIN32)
    SET(OpenCV2_INCLUDE_DIRS
        ${OpenCV2_ROOT_DIR}/include
        ${OpenCV2_INCLUDE_DIRS}
        )
    SET(OpenCV2_LIBRARIES
        ${OpenCV2_LIBRARIES}
        ${OpenCV2_FFMPEG_LIBRARY}
        ${OpenCV2_TS_LIBRARY}
        )
ENDIF(WIN32)

SET(OpenCV2_FOUND ON)
FOREACH(NAME ${OpenCV2_INCLUDE_DIRS})
    IF(NOT EXISTS ${NAME})
        SET(OpenCV2_FOUND OFF)
    ENDIF(NOT EXISTS ${NAME})
ENDFOREACH(NAME)
FOREACH(NAME ${OpenCV2_LIBRARIES})
    IF(NOT EXISTS ${NAME})
        SET(OpenCV2_FOUND OFF)
    ENDIF(NOT EXISTS ${NAME})
ENDFOREACH(NAME)

MARK_AS_ADVANCED(FORCE
                 OpenCV2_ROOT_DIR
                 OpenCV2_CORE_INCLUDE_DIR
                 OpenCV2_IMGPROC_INCLUDE_DIR
                 OpenCV2_FEATURES2D_INCLUDE_DIR
                 OpenCV2_NONFREE_INCLUDE_DIR
                 OpenCV2_FLANN_INCLUDE_DIR
                 OpenCV2_CALIB3D_INCLUDE_DIR
                 OpenCV2_OBJDETECT_INCLUDE_DIR
                 OpenCV2_LEGACY_INCLUDE_DIR
                 OpenCV2_CONTRIB_INCLUDE_DIR
                 OpenCV2_HIGHGUI_INCLUDE_DIR
                 OpenCV2_ML_INCLUDE_DIR
                 OpenCV2_VIDEO_INCLUDE_DIR
                 OpenCV2_GPU_INCLUDE_DIR
                 OpenCV2_CORE_LIBRARY
                 OpenCV2_IMGPROC_LIBRARY
                 OpenCV2_FEATURES2D_LIBRARY
                 OpenCV2_NONFREE_LIBRARY
                 OpenCV2_FLANN_LIBRARY
                 OpenCV2_CALIB3D_LIBRARY
                 OpenCV2_OBJDETECT_LIBRARY
                 OpenCV2_LEGACY_LIBRARY
                 OpenCV2_CONTRIB_LIBRARY
                 OpenCV2_HIGHGUI_LIBRARY
                 OpenCV2_ML_LIBRARY
                 OpenCV2_VIDEO_LIBRARY
                 OpenCV2_GPU_LIBRARY
                 )
IF(WIN32)
    MARK_AS_ADVANCED(FORCE
                     OpenCV2_FFMPEG_LIBRARY
                     OpenCV2_TS_LIBRARY
                     )
ENDIF(WIN32)

# display help message
IF(NOT OpenCV2_FOUND)
    # make FIND_PACKAGE friendly
    IF(OpenCV2_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "OpenCV 2 not found. Please specify it's location with the OpenCV2_ROOT_DIR env. variable.")
    ELSE(OpenCV2_FIND_REQUIRED)
        MESSAGE(STATUS "OpenCV 2 not found.")
    ENDIF(OpenCV2_FIND_REQUIRED)
ENDIF(NOT OpenCV2_FOUND)


