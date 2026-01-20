if(NANOBIND_BINDINGS)
  message(STATUS "[${PROJECT_NAME}] Building Python bindings")
  jrl_find_python(3.8 REQUIRED COMPONENTS Interpreter Development.Module)
  jrl_find_nanobind(2.4.0 CONFIG REQUIRED)
  jrl_find_package(nanoeigenpy CONFIG REQUIRED)
  add_subdirectory(nanobind)

  # Adds a target to build docs using Sphinx The sphinx mode is any mode supported by
  # Sphinx, e.g., html, latexpdf, etc.
  function(add_sphinx_target sphinx_mode)
    add_custom_target(
      mc_rtc-nanobind-docs
      COMMAND make html
      WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/nanobind/docs"
      COMMENT "Build Sphinx ${sphinx_mode} documentation for nanobind"
    )
  endfunction()
  add_sphinx_target(html)
elseif(PYTHON_BINDING)
  add_subdirectory(binding/python)
endif()
