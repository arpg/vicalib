cmake_minimum_required(VERSION 3.4)

#set(CMAKE_USER_MAKE_RULES_OVERRIDE
#   ${CMAKE_CURRENT_SOURCE_DIR}/CMake/c_flag_overrides.cmake CACHE INTERNAL "")
#set(CMAKE_USER_MAKE_RULES_OVERRIDE_CXX
#   ${CMAKE_CURRENT_SOURCE_DIR}/CMake/cxx_flag_overrides.cmake CACHE INTERNAL "")

project(vicalib-cmakeall)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")
find_package(CMakeAll REQUIRED)

SET(CMakeAll_VERIFY_URLS OFF CACHE BOOL "" FORCE)
SET(CMakeAll_RESOLVE_DEPENDENCIES ON CACHE BOOL "" FORCE)

if(UNIX)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -DEIGEN_USE_MKL_ALL")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DEIGEN_USE_MKL_ALL")
endif()

if(WIN32)
  set(MKL_C_FLAGS "${CMAKE_C_FLAGS} -DEIGEN_USE_MKL_ALL" CACHE PATH "" FORCE)
  set(MKL_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_USE_MKL_ALL" CACHE PATH "" FORCE)
endif()

add_compile_options(-march=atom)

find_program(CCACHE NAMES ccache sccache)
if (CCACHE)
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
  message(STATUS "Caching build with ${CCACHE}")
else()
  message(WARNING "Repeated builds will be slow without ccache or sccache installed")
endif()

if (APPLE)
  set(CMAKE_INSTALL_RPATH '@executable_path/../lib' CACHE PATH "" FORCE)
else()
  set(CMAKE_INSTALL_RPATH          '$ORIGIN/../lib' CACHE PATH "" FORCE)
endif()

if(UNIX)
  set(BLAS_LIBRARIES "/opt/intel/mkl/lib/intel64/libmkl_intel_lp64.a")
  list(APPEND BLAS_LIBRARIES "/opt/intel/mkl/lib/intel64/libmkl_core.a"
                             "/opt/intel/mkl/lib/intel64/libmkl_sequential.a")
                             #"/opt/intel/mkl/lib/intel64/libmkl_avx.a")
  set(BLAS_LIBRARIES ${BLAS_LIBRARIES} ${BLAS_LIBRARIES} ${BLAS_LIBRARIES})
  set(LAPACK_INCLUDE_DIR "/opt/intel/mkl/include")
endif()

if(WIN32)
  set(BLAS_LIBRARIES "C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/mkl/lib/intel64/mkl_core.lib")
  list(APPEND BLAS_LIBRARIES "C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/mkl/lib/intel64/mkl_intel_lp64.lib"
                             "C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/mkl/lib/intel64/mkl_sequential.lib")
  set(LAPACK_INCLUDE_DIR "C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/mkl/include")
endif()


set(PROJECTS GFlags GLog Eigen SuiteSparse Ceres protobuf OpenCV Calibu Sophus tinyxml2 HAL vicalib rapidjson POSIX_shim)

if(NOT ANDROID AND BUILD_GUI)
  list(APPEND PROJECTS Pangolin GLEW FreeGLUT)
endif()

if(WIN32)
  list(APPEND PROJECTS dirent dlfcn-win32)
endif()

cma_add_projects(
  ${PROJECTS}
  PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/Projects/"
  SUFFIX ".cmake")


cma_configure_projects()
#cma_configure_launcher()

#cma_print_projects()
