# Set the minimum CMake version and policies for highest tested version
cmake_minimum_required(VERSION 3.15...4.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

project(mc_rtc)

# Warn if the user invokes CMake directly
if(NOT SKBUILD)
  message(
    WARNING
      "\
  This CMake file is meant to be executed using 'scikit-build-core'.
  Running it directly will almost certainly not produce the desired
  result. If you are a user trying to install this package, use the
  command below, which will install all necessary build dependencies,
  compile the package in an isolated environment, and then install it.
  =====================================================================
   $ pip install .
  =====================================================================
  If you are a software developer, and this is your own package, then
  it is usually much more efficient to install the build dependencies
  in your environment once and use the following command that avoids
  a costly creation of a new virtual environment at every compilation:
  =====================================================================
   $ pip install nanobind scikit-build-core[pyproject]
   $ pip install --no-build-isolation -ve .
  =====================================================================
  You may optionally add -Ceditable.rebuild=true to auto-rebuild when
  the package is imported. Otherwise, you need to rerun the above
  after editing C++ files."
  )
endif()

# Try to import all Python components potentially needed by nanobind
find_package(
  Python 3.8 REQUIRED
  COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule
)

# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

# We are now ready to compile the actual extension module
nanobind_add_module(
  # Name of the extension
  _mc_rtc
  # Target the stable ABI for Python 3.12+, which reduces the number of binary wheels
  # that must be built. This does nothing on older Python versions
  STABLE_ABI
  # Build libnanobind statically and merge it into the extension (which itself remains a
  # shared library)
  #
  # If your project builds multiple extensions, you can replace this flag by NB_SHARED
  # to conserve space by reusing a shared libnanobind across libraries
  NB_STATIC
  # Main mc_rtc module
  src/mc_rtc_module/mc_rtc_module.cpp
  # mc_rtc.mc_rtc submodule
  src/mc_rtc_module/mc_rtc/bind_Configuration.cpp
  # mc_rtc.mc_rbdyn submodule
  src/mc_rtc_module/mc_rbdyn/mc_rbdyn_module.cpp
  src/mc_rtc_module/mc_rbdyn/bind_ForceSensor.cpp
  src/mc_rtc_module/mc_rbdyn/bind_JointSensor.cpp
  src/mc_rtc_module/mc_rbdyn/bind_RobotModule.cpp
  src/mc_rtc_module/mc_rbdyn/bind_Device.cpp
  src/mc_rtc_module/mc_rbdyn/bind_RobotData.cpp
  src/mc_rtc_module/mc_rbdyn/bind_RobotFrame.cpp
  src/mc_rtc_module/mc_rbdyn/bind_Robots.cpp
  src/mc_rtc_module/mc_rbdyn/bind_Robot.cpp
  # mc_rtc.mc_control submodule
  src/mc_rtc_module/mc_control/mc_control_module.cpp
  src/mc_rtc_module/mc_control/bind_Gripper.cpp
)

find_package(mc_rtc REQUIRED)
target_link_libraries(_mc_rtc PUBLIC mc_rtc::mc_rtc_utils mc_rtc::mc_rbdyn)
target_include_directories(
  _mc_rtc PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# Install directive for scikit-build-core
install(TARGETS _mc_rtc LIBRARY DESTINATION mc_rtc)

# Display output of stub generation
set(CMAKE_EXECUTE_PROCESS_COMMAND_ECHO STDOUT)

nanobind_add_stub(
  mc_rtc_stub
  MODULE
  mc_rtc
  OUTPUT
  ${SKBUILD_PLATLIB_DIR}/mc_rtc/__init__.pyi
  MARKER_FILE
  ${SKBUILD_PLATLIB_DIR}/mc_rtc/py.typed
  PYTHON_PATH
  ${SKBUILD_PLATLIB_DIR}/mc_rtc/
  DEPENDS
  _mc_rtc
  VERBOSE
  INSTALL_TIME
)

set(MC_RTC_MODULES mc_rtc mc_rbdyn mc_control)

# Generate stub for each submodule
foreach(MODULE IN LISTS MC_RTC_MODULES)
  message(STATUS "setting up stubs for module ${MODULE}")
  nanobind_add_stub(
    mc_rtc_${MODULE}_stub
    MODULE
    mc_rtc.${MODULE}
    OUTPUT
    ${SKBUILD_PLATLIB_DIR}/mc_rtc/${MODULE}/__init__.pyi
    PYTHON_PATH
    ${SKBUILD_PLATLIB_DIR}/mc_rtc/
    DEPENDS
    _mc_rtc
    INSTALL_TIME
  )
endforeach()

# nanobind_add_stub( mc_rtc_mc_rtc_stub MODULE mc_rtc.mc_rtc OUTPUT
# ${SKBUILD_PLATLIB_DIR}/mc_rtc/mc_rtc/__init__.pyi PYTHON_PATH
# ${SKBUILD_PLATLIB_DIR}/mc_rtc/ DEPENDS _mc_rtc INSTALL_TIME )
#
# nanobind_add_stub( mc_rtc_mc_rbdyn_stub MODULE mc_rtc.mc_rbdyn OUTPUT
# ${SKBUILD_PLATLIB_DIR}/mc_rtc/mc_rbdyn/__init__.pyi PYTHON_PATH
# ${SKBUILD_PLATLIB_DIR}/mc_rtc/ DEPENDS _mc_rtc INSTALL_TIME )
