set(JSON_DOC_INSTALL ${CMAKE_INSTALL_DOCDIR}/json)

if(_INSTALL_DOC)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/index.html
          DESTINATION ${JSON_DOC_INSTALL})
  foreach(dir css docson examples fonts js)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${dir}
            DESTINATION ${JSON_DOC_INSTALL})
  endforeach()
endif()
# Install at least the JSON schemas
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/schemas
  DESTINATION ${JSON_DOC_INSTALL}
  FILES_MATCHING PATTERN "*.json")
