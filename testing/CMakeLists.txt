# create script to run tests
#configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/BuildInfo.in
#  ${CMAKE_CURRENT_BINARY_DIR}/BuildInfo.yaml @ONLY IMMEDIATE )

#file( COPY test_rslam.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

#find_package(OpenCV2)
#find_package(Protobuf)
include(FindGTest)
find_package(CANDLE REQUIRED)
find_package(Pangolin REQUIRED)
find_package(SceneGraph REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${CANDLE_INCLUDE_DIRS}
                    ${Sophus_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${SceneGraph_INCLUDE_DIRS} )

if(NOT TARGET gtest)
  add_subdirectory(gtest)
endif()

include_directories(gtest/include)

set(TEST_SOURCES
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
