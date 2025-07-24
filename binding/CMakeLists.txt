if(NANOBIND_BINDINGS)
  # NOTE: requires pip install nanobind scikit-build-core
  add_custom_target(
    mc_rtc-nanobind-bindings ALL
    COMMAND pip install --no-build-isolation -ve .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Building and installing nanobind bindings for mc_rtc (incremental)"
  )

  add_custom_target(
    force-mc_rtc-nanobind-bindings
    COMMAND pip install .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Installing nanobind bindings for mc_rtc (isolated)"
  )

  add_dependencies(mc_rtc-nanobind-bindings ${MC_RTC_ALL_TARGETS})
  add_dependencies(force-mc_rtc-nanobind-bindings ${MC_RTC_ALL_TARGETS})

  add_custom_target(
    mc_rtc-nanobind-docs ALL
    COMMAND make html
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind/docs
    COMMENT "Building nanobind sphinx documentation for mc_rtc"
  )
  # XXX: This should also run if any of the documentation files in docs changes
  add_dependencies(
    mc_rtc-nanobind-docs mc_rtc-nanobind-bindings force-mc_rtc-nanobind-bindings
  )
elseif(PYTHON_BINDING)
  add_subdirectory(binding/python)
endif()
