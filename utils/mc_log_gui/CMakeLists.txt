#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

execute_process(COMMAND ${MC_LOG_UI_PYTHON_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(prefix = '${CMAKE_INSTALL_PREFIX}', plat_specific = True))" RESULT_VARIABLE PYTHON_INSTALL_DIR_FOUND OUTPUT_VARIABLE PYTHON_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)

if("${PYTHON_INSTALL_DIR_FOUND}" EQUAL 0)
  install(DIRECTORY mc_log_ui
    DESTINATION "${PYTHON_INSTALL_DIR}"
    FILES_MATCHING PATTERN "*.py" PATTERN "*.png")
  set(MC_LOG_UI "${CMAKE_CURRENT_BINARY_DIR}/mc_log_ui.py")
  set(PLOT_LOGS "${CMAKE_CURRENT_BINARY_DIR}/plots_log.py")
  configure_file(mc_log_ui.in.py "${MC_LOG_UI}")
  configure_file(plot_logs.in.py "${PLOT_LOGS}")
  if(NOT WIN32)
    install(PROGRAMS "${MC_LOG_UI}" DESTINATION bin RENAME mc_log_ui)
    install(PROGRAMS "${PLOT_LOGS}" DESTINATION bin RENAME mc_plot_logs)
    install(FILES mc_log_ui/icons/icon.png DESTINATION share/icons/hicolor/512x512/apps/ RENAME mc_log_ui.png)
    install(FILES mc_log_ui.desktop DESTINATION share/applications)
  else()
    install(PROGRAMS "${MC_LOG_UI}" DESTINATION bin)
    install(PROGRAMS "${PLOT_LOGS}" DESTINATION bin RENAME mc_plot_logs.py)
  endif()
else()
  message(WARNING "Could not determine where to install Python utilities")
endif()
