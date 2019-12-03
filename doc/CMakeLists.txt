#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(JSON_DOC_INSTALL ${CMAKE_INSTALL_DOCDIR}/json)
configure_file(_plugins/doxygen.rb.in "${CMAKE_CURRENT_SOURCE_DIR}/_plugins/doxygen.rb")

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/_data/schemas
  DESTINATION ${JSON_DOC_INSTALL}
  FILES_MATCHING PATTERN "*.json")
