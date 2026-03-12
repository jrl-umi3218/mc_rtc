#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

configure_file(
  _plugins/data_page_generator.in.rb
  "${CMAKE_CURRENT_BINARY_DIR}/_plugins/data_page_generator.rb"
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/_plugins/data_page_generator.rb
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/_plugins
)

configure_file(_plugins/doxygen.in.rb "${CMAKE_CURRENT_BINARY_DIR}/_plugins/doxygen.rb")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/_plugins/doxygen.rb
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/_plugins
)

file(
  COPY "${CMAKE_CURRENT_SOURCE_DIR}/_plugins/"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/_plugins/"
  FILES_MATCHING
  PATTERN "*.rb"
  PATTERN "*.in.rb" EXCLUDE
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/_data/schemas
  DESTINATION ${JSON_DOC_INSTALL}
  FILES_MATCHING
  PATTERN "*.json"
)
