function(git)
  find_package(Git QUIET REQUIRED)
  execute_process(COMMAND ${GIT_EXECUTABLE} ${ARGN} RESULT_VARIABLE RESULT OUTPUT_QUIET ERROR_VARIABLE ERROR)
  if(RESULT)
    message(FATAL_ERROR "${ERROR}")
  endif()
endfunction()

if(NOT CMakeAll_DIR)
  set(CMakeAll_DIR ${PROJECT_BINARY_DIR}/CMakeAll)
  if(NOT EXISTS ${CMakeAll_DIR})
    git(clone https://github.com/auneri/CMakeAll ${CMakeAll_DIR})
  endif()
  if(CMakeAll_FIND_VERSION)
    git(checkout v${CMakeAll_FIND_VERSION} WORKING_DIRECTORY ${CMakeAll_DIR})
  else()
    message(STATUS "Updating CMakeAll master")
    git(checkout master WORKING_DIRECTORY ${CMakeAll_DIR})
    git(pull WORKING_DIRECTORY ${CMakeAll_DIR})
  endif()
endif()

find_package(CMakeAll ${CMakeAll_FIND_VERSION} HINTS ${CMakeAll_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CMakeAll
  FOUND_VAR CMakeAll_FOUND
  REQUIRED_VARS CMakeAll_DIR CMakeAll_FOUND)
