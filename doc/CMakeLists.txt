#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

if(INSTALL_DOCUMENTATION)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/index.html
                ${CMAKE_CURRENT_SOURCE_DIR}/robots.txt
          DESTINATION ${CMAKE_INSTALL_DOCDIR})
endif()

add_subdirectory(json)
