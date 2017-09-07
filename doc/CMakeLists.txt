if(_INSTALL_DOC)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/index.html
                ${CMAKE_CURRENT_SOURCE_DIR}/robots.txt
          DESTINATION ${CMAKE_INSTALL_DOCDIR})
endif()

add_subdirectory(json)
