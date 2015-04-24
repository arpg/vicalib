# FindAndroidKernel.cmake
#   AndroidKernel_FOUND
#   AndroidKernel_INCLUDE_DIRS
#   AndroidKernel_LIBRARIES
#
# Copy android kernel headers onto ${CMAKE_FIND_ROOT_PATH} path
# e.g. TOOLCHAIN_DIR/user/kernel

SET(AndroidKernel_POSSIBLE_ROOT_DIRS
    /kernel/frameworks/av/include
    /kernel/frameworks/native/include
    /kernel/system/core/include
    /kernel/system/media/camera/include
    /kernel/hardware/libhardware/include
)

FIND_PATH(AndroidKernel_AV_INCLUDE_DIR
          NAMES camera/ICamera.h gestures/IGestureDevice.h media/IMediaPlayer.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})
FIND_PATH(AndroidKernel_NATIVE_INCLUDE_DIR
          NAMES gui/IGraphicBufferProducer.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})
FIND_PATH(AndroidKernel_HARDWARE_INCLUDE_DIR
          NAMES hardware/camera.h hardware/lights.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})
FIND_PATH(AndroidKernel_CORE_INCLUDE_DIR
          NAMES cutils/atomic.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})
FIND_PATH(AndroidKernel_CAMERA_INCLUDE_DIR
          NAMES system/camera_metadata.h
          PATHS ${AndroidKernel_POSSIBLE_ROOT_DIRS})

SET(AndroidKernel_INCLUDE_DIRS
    ${AndroidKernel_AV_INCLUDE_DIR}
    ${AndroidKernel_NATIVE_INCLUDE_DIR}
    ${AndroidKernel_HARDWARE_INCLUDE_DIR}
    ${AndroidKernel_CORE_INCLUDE_DIR}
    ${AndroidKernel_CAMERA_INCLUDE_DIR}
    )

SET(AndroidKernel_LIBRARIES
    cameraservice camera_client
    utils cutils
    gui binder
)

SET(AndroidKernel_FOUND ON)

FOREACH(NAME ${AndroidKernel_INCLUDE_DIRS})
    IF(NOT EXISTS ${NAME})
        SET(AndroidKernel_FOUND OFF)
    ENDIF(NOT EXISTS ${NAME})
ENDFOREACH(NAME)

MARK_AS_ADVANCED(FORCE
                 AndroidKernel_AV_INCLUDE_DIR
                 AndroidKernel_NATIVE_INCLUDE_DIR
                 AndroidKernel_HARDWARE_INCLUDE_DIR
                 AndroidKernel_CORE_INCLUDE_DIR
		 AndroidKernel_CAMERA_INCLUDE_DIR
                 )

IF(AndroidKernel_FOUND)
   IF(NOT AndroidKernel_FIND_QUIETLY)
      MESSAGE(STATUS "Found Android Kernel headers.")
   ENDIF (NOT AndroidKernel_FIND_QUIETLY)
ELSE(AndroidKernel_FOUND)
   IF(AndroidKernel_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find AndroidKernel. Please specify it's location.")
   ENDIF(AndroidKernel_FIND_REQUIRED)
ENDIF(AndroidKernel_FOUND)
