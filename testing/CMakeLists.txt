include(FindGTest)
find_package(VICALIB REQUIRED)
find_package(Pangolin REQUIRED)
find_package(SceneGraph REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${VICALIB_INCLUDE_DIRS}
                    ${Sophus_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${SceneGraph_INCLUDE_DIRS} )

if(NOT TARGET gtest)
  add_subdirectory(gtest)
endif()

include_directories(gtest/include)

set(TEST_SOURCES
    vi_sim_test.cpp
    )

include(def_test)

def_executable(test_main
  SOURCES test_main.cpp ${TEST_SOURCES}
  DEPENDS
  vicalib gtest
  LINK_LIBS
  ${SceneGraph_LIBRARIES}
  ${Pangolin_LIBRARIES}
  )

