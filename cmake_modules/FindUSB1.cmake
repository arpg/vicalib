# - Try to find USB1
# Once done this will define
#
#  USB1_FOUND - system has USB1
#  USB1_INCLUDE_DIRS - the USB1 include directory
#  USB1_LIBRARIES - Link these to use USB1
#  USB1_DEFINITIONS - Compiler switches required for using USB1
#
#  Copyright (c) 2006 Andreas Schneider <mail@cynapses.org>
#  Adapted for libusb1.0 by Cory Quammen <cquammen@cs.unc.edu>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#


if(USB1_LIBRARIES AND USB1_INCLUDE_DIRS)
  # in cache already
  set(USB1_FOUND TRUE)
else(USB1_LIBRARIES AND USB1_INCLUDE_DIRS)
  find_path(USB1_INCLUDE_DIR
    NAMES
      libusb.h
    PATHS
      /usr/include
      /usr/include/libusb-1.0
      /usr/local/include
      /opt/local/include/libusb-1.0/
      /opt/local/include
      /sw/include
  )

  find_library(USB1_LIBRARY
    NAMES
      usb-1.0
    PATHS
      /usr/lib
      /usr/lib64
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(USB1_INCLUDE_DIRS
    ${USB1_INCLUDE_DIR}
  )
  set(USB1_LIBRARIES
    ${USB1_LIBRARY}
)

  if(USB1_INCLUDE_DIRS AND USB1_LIBRARIES)
     set(USB1_FOUND TRUE)
  endif(USB1_INCLUDE_DIRS AND USB1_LIBRARIES)

  if(USB1_FOUND)
    if(NOT USB1_FIND_QUIETLY)
      message(STATUS "Found USB1: ${USB1_LIBRARIES}")
    endif(NOT USB1_FIND_QUIETLY)
  else(USB1_FOUND)
    if(USB1_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find USB1")
    endif(USB1_FIND_REQUIRED)
  endif(USB1_FOUND)

  # show the USB1_INCLUDE_DIRS and USB1_LIBRARIES variables only in the advanced view
  mark_as_advanced(USB1_INCLUDE_DIRS USB1_LIBRARIES)

endif(USB1_LIBRARIES AND USB1_INCLUDE_DIRS)

