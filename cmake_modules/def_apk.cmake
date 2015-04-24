include(CMakeParseArguments)
# Declare an APK w/
#   - LINK_LIBS: Library dependencies for NDK lib
#   - DEPENDS: Targets to link against and depend on
#   - JNI_SOURCES: JNI Sources
#   - JNI_LIB: Name for JNI Library
#   - CONDITIONS: Conditions for building this apk
function(def_apk apk)
  if (NOT ANDROID)
    return()
  endif()
  string(TOUPPER ${apk} APK)

  set(APK_OPTIONS)
  set(APK_SINGLE_ARGS JNI_DIR JNI_LIB)
  set(APK_MULTI_ARGS DEPENDS CONDITIONS LINK_LIBS JNI_SOURCES)
  cmake_parse_arguments(apk
    "${APK_OPTIONS}"
    "${APK_SINGLE_ARGS}"
    "${APK_MULTI_ARGS}"
    "${ARGN}"
    )

  set(cache_var BUILD_${APK})
  set(${cache_var} ON CACHE BOOL "Enable ${APK} apk compilation.")

  if(apk_CONDITIONS)
    foreach(cond ${apk_CONDITIONS})
      if(NOT ${cond})
	set(${cache_var} OFF)
	message("${cache_var} is false because ${cond} is false.")
	return()
      endif()
    endforeach()
  endif()

  if(apk_DEPENDS)
    foreach(dep ${apk_DEPENDS})
      if(NOT TARGET ${dep})
	set(${cache_var} OFF)
	message("${cache_var} is false because ${dep} is not being built.")
	return()
      endif()
    endforeach()
  endif()

  set(build_type_cache_var ${APK}_BUILD_TYPE)
  set(${build_type_cache_var} "" CACHE STRING
    "Target specific build configuration for ${apk} APK")

  if(apk_JNI_SOURCES AND NOT apk_JNI_DIR)
    message(FATAL_ERROR "JNI_SOURCES given to def_apk for ${apk}, but no "
      "JNI_DIR listed. JNI_DIR is required")
  endif()

  if(${cache_var})
    if(apk_JNI_DIR)
      if(NOT apk_JNI_LIB)
	message(FATAL_ERROR "JNI_LIB must be given to def_apk.")
      endif()
      find_library(GNUSTL_SHARED_LIBRARY gnustl_shared)

      if(NOT GNUSTL_SHARED_LIBRARY)
	message(FATAL_ERROR "Could not find required GNU STL shared library.")
      endif()
      list(APPEND apk_LINK_LIBS ${GNUSTL_SHARED_LIBRARY})
      set(android_mk ${apk_JNI_DIR}/Android.mk)

      macro(mk_append txt)
	file(APPEND ${android_mk} "${txt}\n")
      endmacro()

      macro(clear_vars)
	mk_append("include $(CLEAR_VARS)")
      endmacro()

      file(WRITE ${android_mk} "LOCAL_PATH := $(call my-dir)\n\n")

      # Generate Android.mk in build dir
      foreach(dep ${apk_DEPENDS})
	# - Add each dependent library's full path
	clear_vars()
	mk_append("LOCAL_MODULE  := ${dep}")
	get_target_property(dep_path ${dep} LOCATION)
	mk_append("LOCAL_SRC_FILES := ${dep_path}")
	mk_append("include $(PREBUILT_SHARED_LIBRARY)\n")
      endforeach()

      foreach(lib ${apk_LINK_LIBS})
	clear_vars()
	get_filename_component(module_name ${lib} NAME_WE)
	get_filename_component(lib_path ${lib} REALPATH)
	mk_append("LOCAL_MODULE  := ${module_name}")
	mk_append("LOCAL_SRC_FILES := ${lib_path}")
	mk_append("include $(PREBUILT_SHARED_LIBRARY)\n")
      endforeach()

      clear_vars()

      # - Add all include directories
      # - Add compile flags for C, CXX

      # - Add JNI sources
      mk_append("LOCAL_MODULE := ${apk_JNI_LIB}")
      foreach(jni_src ${apk_JNI_SOURCES})
	mk_append("LOCAL_SRC_FILES += ${jni_src}")
      endforeach()

      get_directory_property(include_dirs INCLUDE_DIRECTORIES)
      foreach(dir ${include_dirs})
	mk_append("LOCAL_C_INCLUDES += ${dir}")
      endforeach()

      foreach(dep ${apk_DEPENDS})
	mk_append("LOCAL_SHARED_LIBRARIES += ${dep}")
      endforeach()

      foreach(lib ${apk_LINK_LIBS})
	get_filename_component(lib_name ${lib} NAME_WE)
	mk_append("LOCAL_SHARED_LIBRARIES += ${lib_name}")
      endforeach()

      # Only alter the compile flags if the build type is set
      string(TOUPPER "${${build_type_cache_var}}" APK_BUILD_TYPE)
      if (APK_BUILD_TYPE)
	set(build_type ${APK_BUILD_TYPE})
      else()
	string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type)
      endif()

      set(c_flags ${CMAKE_C_FLAGS_${build_type}})
      set(cxx_flags ${CMAKE_CXX_FLAGS_${build_type}})

      mk_append("LOCAL_LDLIBS := -llog")
      mk_append("LOCAL_CFLAGS := ${c_flags}")
      mk_append("LOCAL_CPPFLAGS := ${cxx_flags}")
      mk_append("include $(BUILD_SHARED_LIBRARY)")
    endif()

    # Run ndk-build in Android.mk dir
    add_custom_target(${apk}-jni
      COMMAND ndk-build
      DEPENDS ${apk_DEPENDS}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      )

    macro(apk_type type)
      add_custom_target(${apk}-${type}
	COMMAND ant ${type}
	DEPENDS ${apk}-jni
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

      string(REGEX MATCH "." first_letter ${type})
      add_custom_target(${apk}-install${first_letter}
	COMMAND ant install${first_letter}
	DEPENDS ${apk}-${type}
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endmacro()

    apk_type(debug)
    apk_type(release)

    add_custom_target(${apk} ALL DEPENDS ${apk}-debug)
  endif()
endfunction()