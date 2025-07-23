if(NANOBIND_BINDINGS)
  # NOTE: requires pip install nanobind scikit-build-core
  add_custom_target(
    nanobind-bindings ALL
    COMMAND pip install --no-build-isolation -ve .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Building and installing nanobind bindings for mc_rtc (incremental)"
  )

  add_custom_target(
    force-nanobind-bindings
    COMMAND pip install .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Installing nanobind bindings for mc_rtc (isolated)"
  )

  message(WARNING "ALL_MC_RTC_TARGETS: ${ALL_MC_RTC_TARGETS}")
  add_dependencies(nanobind-bindings ${ALL_MC_RTC_TARGETS})
  add_dependencies(force-nanobind-bindings ${ALL_MC_RTC_TARGETS})
elseif(PYTHON_BINDING)
  add_subdirectory(binding/python)
endif()
