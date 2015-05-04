cmake_minimum_required(VERSION 2.8)
find_package(VICALIB REQUIRED)
find_package(Pangolin REQUIRED)
find_package(SceneGraph REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${VICALIB_INCLUDE_DIRS}
                    ${Sophus_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${SceneGraph_INCLUDE_DIRS} )

def_executable(vicalib_exec
  SOURCES main.cc
  DEPENDS
  vicalib
  LINK_LIBS
  ${SceneGraph_LIBRARIES}
  ${Pangolin_LIBRARIES}
  )

install(TARGETS vicalib_exec RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
