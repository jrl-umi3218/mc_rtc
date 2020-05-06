#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

execute_process(COMMAND "python" -c "from distutils import sysconfig; print(sysconfig.get_python_lib(prefix = '${CMAKE_INSTALL_PREFIX}', plat_specific = True))" RESULT_VARIABLE PYTHON_INSTALL_DIR_FOUND OUTPUT_VARIABLE PYTHON_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)

if("${PYTHON_INSTALL_DIR_FOUND}" EQUAL 0)
  install(DIRECTORY mc_log_ui
    DESTINATION "${PYTHON_INSTALL_DIR}"
    FILES_MATCHING PATTERN "*.py" PATTERN "*.png")
  if(NOT WIN32)
    install(PROGRAMS mc_log_ui.py DESTINATION bin RENAME mc_log_ui)
    install(PROGRAMS plot_logs.py DESTINATION bin RENAME mc_plot_logs)
    install(FILES mc_log_ui/icons/icon.png DESTINATION share/icons/hicolor/512x512/apps/ RENAME mc_log_ui.png)
    install(FILES mc_log_ui.desktop DESTINATION share/applications)
  else()
    install(PROGRAMS mc_log_ui.py DESTINATION bin)
    install(PROGRAMS plot_logs.py DESTINATION bin RENAME mc_plot_logs.py)
  endif()
else()
  message(WARNING "Could not determine where to install Python utilities")
endif()
