set(EP_TAG 3.2.10)
set(EP_URL "https://github.com/eigenteam/eigen-git-mirror.git")

cma_end_definition()

ExternalProject_Add(${EP_NAME}
  GIT_REPOSITORY  ${EP_URL}
  GIT_TAG ${EP_TAG}
  UPDATE_DISCONNECTED 1
  SOURCE_DIR ${PROJECT_BINARY_DIR}/src/${EP_NAME}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "")

set(EIGEN_INCLUDE_DIR "${PROJECT_BINARY_DIR}/src/${EP_NAME}" CACHE INTERNAL "")
