set(EP_URL "https://github.com/miloyip/rapidjson/")

cma_end_definition()

ExternalProject_Add(${EP_NAME}
  GIT_REPOSITORY  ${EP_URL}
  GIT_TAG 0f96b5605a5279bdc1831500de45e89ee516380b
  SOURCE_DIR ${PROJECT_BINARY_DIR}/src/${EP_NAME}
  UPDATE_DISCONNECTED 1
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "")

set(${EP_NAME}_INCLUDE_DIR "${PROJECT_BINARY_DIR}/src/${EP_NAME}/include/" CACHE INTERNAL "")
