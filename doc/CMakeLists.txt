#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

configure_file(_plugins/doxygen.rb.in "${CMAKE_CURRENT_SOURCE_DIR}/_plugins/doxygen.rb")

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/_data/schemas/
  DESTINATION ${MC_RTC_JSON_SCHEMA_PATH}
  FILES_MATCHING
  PATTERN "*.json"
)
