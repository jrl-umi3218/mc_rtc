set(JSON_DOC_INSTALL ${CMAKE_INSTALL_DOCDIR}/json)

if(_INSTALL_DOC)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/index.html
          DESTINATION ${JSON_DOC_INSTALL})
  foreach(dir css docson examples fonts js schemas)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${dir}
            DESTINATION ${JSON_DOC_INSTALL})
  endforeach()
endif()
