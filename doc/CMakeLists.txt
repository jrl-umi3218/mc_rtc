#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(MC_RTC_JEKYLL_PLUGINS_PATH "${CMAKE_INSTALL_DOCDIR}/jekyll/plugins")
configure_file(
  _plugins/data_page_generator.in.rb
  "${CMAKE_CURRENT_BINARY_DIR}/_plugins/data_page_generator.rb"
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/_plugins/data_page_generator.rb
        DESTINATION ${MC_RTC_JEKYLL_PLUGINS_PATH}
)

configure_file(_plugins/doxygen.in.rb "${CMAKE_CURRENT_BINARY_DIR}/_plugins/doxygen.rb")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/_plugins/doxygen.rb
        DESTINATION ${MC_RTC_JEKYLL_PLUGINS_PATH}
)

install(FILES _plugins/include_raw.rb _plugins/link.rb _plugins/pretty_json.rb
        DESTINATION "${MC_RTC_JEKYLL_PLUGINS_PATH}"
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/_data/schemas
  DESTINATION ${CMAKE_INSTALL_DOCDIR}/json
  FILES_MATCHING
  PATTERN "*.json"
)
