if(NANOBIND_BINDINGS)
  # NOTE: requires pip install nanobind scikit-build-core
  if(NOT PYTHON_EXECUTABLE)
    find_package(
      Python3
      COMPONENTS Interpreter
      REQUIRED
    )
    set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
  endif()
  add_custom_target(
    mc_rtc-nanobind-bindings
    COMMAND
      cmake -E env LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH}
      CMAKE_PREFIX_PATH=$ENV{CMAKE_PREFIX_PATH} ${PYTHON_EXECUTABLE} -m pip install
      --no-build-isolation -ve .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Building and installing nanobind bindings for mc_rtc (incremental)"
  )

  add_custom_target(
    force-mc_rtc-nanobind-bindings
    COMMAND ${PYTHON_EXECUTABLE} -m pip install .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Installing nanobind bindings for mc_rtc (isolated)"
  )

  set(SPHINXBUILD "${VIRTUAL_ENV}/bin/sphinx-build")
  set(SOURCEDIR "${CMAKE_CURRENT_LIST_DIR}/nanobind/docs/source")
  set(BUILDDIR "${CMAKE_CURRENT_LIST_DIR}/nanobind/docs/build")
  set(SPHINXOPTS "") # Add options as needed

  # Adds a target to build docs using Sphinx The sphinx mode is any mode supported by
  # Sphinx, e.g., html, latexpdf, etc.
  function(add_sphinx_target sphinx_mode)
    add_custom_target(
      mc_rtc-nanobind-docs
      COMMAND ${SPHINXBUILD} -M ${sphinx_mode} "${SOURCEDIR}" "${BUILDDIR}"
              ${SPHINXOPTS}
      WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/nanobind/docs"
      COMMENT "Build Sphinx ${sphinx_mode} documentation for nanobind"
    )
  endfunction()
  add_sphinx_target(html)
elseif(PYTHON_BINDING)
  add_subdirectory(binding/python)
endif()
