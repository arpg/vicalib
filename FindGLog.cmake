# - Try to find glog (or miniglog for Android)
# Once done, this will define
#
#  GLog_FOUND - system has glog
#  GLog_INCLUDE_DIRS - the glog include directories
#  GLog_LIBRARIES - link these to use glog

# Find header and lib
find_path(GLog_INCLUDE_DIR NAMES glog/logging.h)
find_library(GLog_LIBRARIES NAMES glog)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLog DEFAULT_MSG GLog_INCLUDE_DIR GLog_LIBRARIES)
set(GLOG_INCLUDE_DIRS ${GLog_INCLUDE_DIRS})
set(GLOG_LIBRARIES ${GLog_LIBRARIES})
