#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(BINDINGS_LIBRARIES
  mc_control_fsm
  mc_control
  mc_observers
  mc_rbdyn
  mc_rtc_loader
  mc_rtc_utils
  mc_rtc_gui
  mc_solver
  mc_tasks
  mc_trajectory
  mc_control_client
)
if(WIN32)
  list(APPEND BINDINGS_LIBRARIES ltdl RBDyn::RBDyn RBDyn::Parsers Tasks::Tasks spdlog::spdlog)
endif()

set(PYTHON_BINDING_USE_PYTEST ON CACHE BOOL "" FORCE)

function(make_cython_bindings PACKAGE)
  set(options)
  set(oneValueArgs VERSION)
  set(multiValueArgs MODULES TARGETS EXPORT_SOURCES PRIVATE_SOURCES
                     GENERATE_SOURCES)
  cmake_parse_arguments(CYTHON_BINDINGS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})
  if(NOT DEFINED CYTHON_BINDINGS_VERSION)
    set(CYTHON_BINDINGS_VERSION ${PROJECT_VERSION})
  endif()
  if(NOT DEFINED CYTHON_BINDINGS_EXPORT_SOURCES)
    set(CYTHON_BINDINGS_EXPORT_SOURCES)
  endif()
  if(NOT DEFINED CYTHON_BINDINGS_PRIVATE_SOURCES)
    set(CYTHON_BINDINGS_PRIVATE_SOURCES)
  endif()
  if(NOT DEFINED CYTHON_BINDINGS_GENERATE_SOURCES)
    set(CYTHON_BINDINGS_GENERATE_SOURCES)
  endif()
  if(NOT DEFINED CYTHON_BINDINGS_MODULES)
    set(CYTHON_BINDINGS_MODULES "${PACKAGE}.${PACKAGE}")
  endif()
  if(NOT DEFINED CYTHON_BINDINGS_TARGETS)
    message(
      FATAL_ERROR
        "Error in ADD_CYTHON_BINDINGS, bindings should depend on at least one target"
    )
  endif()
  set(CYTHON_BINDINGS_SOURCES)
  list(APPEND CYTHON_BINDINGS_SOURCES ${CYTHON_BINDINGS_EXPORT_SOURCES})
  list(APPEND CYTHON_BINDINGS_SOURCES ${CYTHON_BINDINGS_PRIVATE_SOURCES})
  list(APPEND CYTHON_BINDINGS_SOURCES ${CYTHON_BINDINGS_GENERATE_SOURCES})
  set(CYTHON_BINDINGS_COMPILE_SOURCES)
  set(WITH_TESTS False)
  foreach(SRC ${CYTHON_BINDINGS_SOURCES})
    if(${SRC} MATCHES "^tests/")
      set(WITH_TESTS True)
    endif()
    if(${SRC} MATCHES ".pyx$")
      list(APPEND CYTHON_BINDINGS_COMPILE_SOURCES ${SRC})
    endif()
  endforeach()
  add_library(_cython_dummy_${PACKAGE} SHARED EXCLUDE_FROM_ALL "${CYTHON_DUMMY_CPP_LOCATION}")
  target_link_libraries(_cython_dummy_${PACKAGE} INTERFACE ${CYTHON_BINDINGS_TARGETS})
  set_target_properties(_cython_dummy_${PACKAGE} PROPERTIES FOLDER "bindings/details")
  foreach(PYTHON ${PYTHON_BINDING_VERSIONS})
    set(PACKAGE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PYTHON}/$<CONFIG>/${PACKAGE})
    if(DEFINED PYTHON_DEB_ROOT)
      set(PYTHON_INSTALL_DESTINATION ${${PYTHON}_SITEARCH})
    else()
      execute_process(COMMAND ${${PYTHON}_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(prefix = '${CMAKE_INSTALL_PREFIX}', plat_specific = True))" RESULT_VARIABLE PYTHON_INSTALL_DESTINATION_FOUND OUTPUT_VARIABLE PYTHON_INSTALL_DESTINATION OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
    foreach(F ${CYTHON_BINDINGS_GENERATE_SOURCES})
      configure_file(${F} ${CMAKE_CURRENT_BINARY_DIR}/${PYTHON}/cmake/${F})
      file(GENERATE
        OUTPUT ${PACKAGE_OUTPUT_DIRECTORY}/${F}
        INPUT ${CMAKE_CURRENT_BINARY_DIR}/${PYTHON}/cmake/${F}
      )
    endforeach()
    foreach(F ${CYTHON_BINDINGS_EXPORT_SOURCES})
      file(GENERATE
        OUTPUT ${PACKAGE_OUTPUT_DIRECTORY}/${F}
        INPUT ${CMAKE_CURRENT_SOURCE_DIR}/${F}
      )
    endforeach()
    foreach(F ${CYTHON_BINDINGS_PRIVATE_SOURCES})
      if(${F} MATCHES "^tests/")
        file(GENERATE
          OUTPUT ${PACKAGE_OUTPUT_DIRECTORY}/${F}
          INPUT ${CMAKE_CURRENT_SOURCE_DIR}/${F}
        )
      endif()
    endforeach()
    install(DIRECTORY ${PACKAGE_OUTPUT_DIRECTORY}/
            DESTINATION ${PYTHON_INSTALL_DESTINATION}
            PATTERN "tests/*" EXCLUDE
            PATTERN ".pytest_cache/*" EXCLUDE
            PATTERN "__pycache__/*" EXCLUDE)
    if(WITH_TESTS AND BUILD_TESTING)
      if(WIN32)
        set(ENV_VAR "PATH")
        set(PATH_SEP ";")
      else()
        set(ENV_VAR "LD_LIBRARY_PATH")
        set(PATH_SEP ":")
      endif()
      set(EXTRA_LD_PATH "")
      foreach(TGT ${CYTHON_BINDINGS_TARGETS})
        _is_interface_library(${TGT} IS_INTERFACE)
        if(NOT ${IS_INTERFACE})
          set(EXTRA_LD_PATH
              "$<TARGET_FILE_DIR:${TGT}>${PATH_SEP}${EXTRA_LD_PATH}")
        endif()
      endforeach()
      add_test(
        NAME test-${PACKAGE}-${PYTHON}-bindings
        COMMAND
          ${CMAKE_COMMAND} -E env "${ENV_VAR}=${EXTRA_LD_PATH}$ENV{${ENV_VAR}}"
          ${CMAKE_COMMAND} -E env "PYTHONPATH=.${PATH_SEP}$ENV{PYTHONPATH}"
          ${CMAKE_COMMAND} -E chdir "${PACKAGE_OUTPUT_DIRECTORY}" ${${PYTHON}_EXECUTABLE} -m pytest)
    endif()
    foreach(MOD ${CYTHON_BINDINGS_MODULES})
      string(REPLACE "." "/" SRC ${MOD})
      set(SRC "${SRC}.pyx")
      if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${SRC}")
        message(FATAL_ERROR "Expected to find ${CMAKE_CURRENT_SOURCE_DIR}/${SRC}")
      endif()
      string(REGEX REPLACE ".pyx$" ".cpp" SRC_CPP ${SRC})
      string(REGEX REPLACE "/[^/]*$" "" SRC_DIR ${SRC})
      string(REGEX REPLACE "^(.*)\\..*$" "\\1" LIB_NAME ${MOD})
      string(REGEX REPLACE "\\." "_" LIB_NAME ${LIB_NAME})
      string(REGEX REPLACE "^.*\\.(.*)$" "\\1" LIB_OUTPUT_NAME ${MOD})
      set(MOD_OUTPUT_DIRECTORY ${PACKAGE_OUTPUT_DIRECTORY}/${SRC_DIR})
      set(CPP_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PYTHON}/${SRC_DIR})
      set(CPP_OUT ${CMAKE_CURRENT_BINARY_DIR}/${PYTHON}/${SRC_CPP})
      file(MAKE_DIRECTORY ${CPP_OUT_DIR})
      add_custom_command(OUTPUT ${CPP_OUT}
        COMMAND ${${PYTHON}_EXECUTABLE} -m cython --cplus -o ${CPP_OUT} "-I$<JOIN:$<REMOVE_DUPLICATES:$<TARGET_PROPERTY:_cython_dummy_${PACKAGE},INCLUDE_DIRECTORIES>>,;-I>" -I${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/${SRC}
        DEPENDS ${CYTHON_BINDINGS_SOURCES} ${CYTHON_BINDINGS_TARGETS}
        COMMAND_EXPAND_LISTS
      )
      set(TARGET_NAME ${LIB_NAME}_${PYTHON})
      if(${PYTHON} STREQUAL "Python")
        Python_add_library(${TARGET_NAME} SHARED ${CPP_OUT})
      elseif(${PYTHON} STREQUAL "Python2")
        Python2_add_library(${TARGET_NAME} SHARED ${CPP_OUT})
      elseif(${PYTHON} STREQUAL "Python3")
        Python3_add_library(${TARGET_NAME} SHARED ${CPP_OUT})
      else()
        message(FATAL_ERROR "Unknown Python value: ${PYTHON}")
      endif()
      target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
      target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/${PYTHON})
      target_link_libraries(${TARGET_NAME} PUBLIC ${CYTHON_BINDINGS_TARGETS} ${PYTHON}::Python ${PYTHON}::NumPy)
      set_target_properties(${TARGET_NAME} PROPERTIES
        CXX_VISIBILITY_PRESET default
        PREFIX ""
        DEBUG_POSTFIX ""
        OUTPUT_NAME ${LIB_OUTPUT_NAME}
        LIBRARY_OUTPUT_DIRECTORY ${MOD_OUTPUT_DIRECTORY}
        RUNTIME_OUTPUT_DIRECTORY ${MOD_OUTPUT_DIRECTORY})
    endforeach()
  endforeach()
endfunction()

make_cython_bindings(mc_rtc
  TARGETS ${BINDINGS_LIBRARIES}
  VERSION ${PROJECT_VERSION}
  MODULES mc_control.mc_control
          mc_control.fsm.fsm
          mc_observers.mc_observers
          mc_rbdyn.mc_rbdyn
          mc_rtc.mc_rtc
          mc_rtc.gui.gui
          mc_solver.mc_solver
          mc_tasks.mc_tasks
          mc_tasks.force.force
  EXPORT_SOURCES
    mc_control/fsm/__init__.py
    mc_control/fsm/c_fsm.pxd
    mc_control/fsm/fsm.pxd
    mc_control/__init__.py
    mc_control/c_mc_control.pxd
    mc_control/mc_control.pxd
    mc_observers/__init__.py
    mc_observers/c_mc_observers.pxd
    mc_observers/mc_observers.pxd
    mc_rbdyn/__init__.py
    mc_rbdyn/c_mc_rbdyn.pxd
    mc_rbdyn/mc_rbdyn.pxd
    mc_rtc/gui/__init__.py
    mc_rtc/gui/c_gui.pxd
    mc_rtc/gui/gui.pxd
    mc_rtc/__init__.py
    mc_rtc/c_mc_rtc.pxd
    mc_rtc/mc_rtc.pxd
    mc_solver/__init__.py
    mc_solver/c_mc_solver.pxd
    mc_solver/mc_solver.pxd
    mc_tasks/force/__init__.py
    mc_tasks/force/c_force.pxd
    mc_tasks/force/force.pxd
    mc_tasks/__init__.py
    mc_tasks/mc_tasks.pxd
    mc_tasks/c_mc_tasks.pxd
  PRIVATE_SOURCES
    include/mc_control_fsm_wrapper.hpp
    include/mc_control_wrapper.hpp
    include/mc_rtc_gui_wrapper.hpp
    include/mc_rtc_wrapper.hpp
    include/mc_solver_wrapper.hpp
    include/mc_rbdyn_wrapper.hpp
    include/mc_tasks_force_wrapper.hpp
    mc_control/fsm/fsm.pyx
    mc_control/mc_control.pyx
    mc_observers/mc_observers.pyx
    mc_rbdyn/mc_rbdyn.pyx
    mc_rtc/gui/gui.pyx
    mc_rtc/mc_rtc.pyx
    mc_solver/mc_solver.pyx
    mc_tasks/force/force.pyx
    mc_tasks/mc_tasks.pyx
    mc_tasks/com_trajectory_task.pxi
    mc_tasks/position_trajectory_task.pxi
    mc_tasks/orientation_trajectory_task.pxi
    mc_tasks/vector_orientation_trajectory_task.pxi
    tests/test_mc_rtc.py
  GENERATE_SOURCES
    tests/test_mc_tasks.py
)

