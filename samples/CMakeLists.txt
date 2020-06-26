macro(add_example NAME)
  add_executable(${NAME} ${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC ${ARGN})
endmacro()

macro(add_example_doc NAME)
  file(RELATIVE_PATH MC_RTC_SAMPLE_PATH ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.in.cpp)
  set(MC_RTC_SAMPLE_COMMAND ${NAME})
  configure_file(${NAME}.in.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp)
  add_example(${NAME} ${ARGN})

  install(TARGETS ${NAME} DESTINATION bin)

  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp
    DESTINATION ${CMAKE_INSTALL_DOCDIR}/doxygen-html/examples
  )
endmacro()

add_subdirectory(CoMGenerator)
