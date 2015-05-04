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

set(TEST_ARCHIVE "vi_sim_resources.tar.gz")
set(TEST_ARCHIVE_URL "http://arpg-web.colorado.edu/~arpg/${TEST_ARCHIVE}")
set(TEST_ARCHIVE_LOCAL "${CMAKE_BINARY_DIR}/testing/${TEST_ARCHIVE}")

if (NOT EXISTS "${TEST_ARCHIVE_LOCAL}" AND NOT EXISTS "${CMAKE_BINARY_DIR}/testing/imu/timestamp.txt")
    file(DOWNLOAD "${TEST_ARCHIVE_URL}" "${TEST_ARCHIVE_LOCAL}")
    message("Downloaded ${TEST_ARCHIVE_LOCAL} from ${TEST_ARCHIVE_URL}.")
endif()

execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${TEST_ARCHIVE_LOCAL}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/testing
)

file(REMOVE ${TEST_ARCHIVE_LOCAL})
