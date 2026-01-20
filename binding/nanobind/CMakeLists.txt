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
  LTO
  # Main mc_rtc module
  src/mc_rtc_module/mc_rtc_module.cpp
  # mc_rtc.mc_rtc submodule
  src/mc_rtc_module/mc_rtc/bind_Configuration.cpp
  src/mc_rtc_module/mc_rtc/bind_Loader.cpp
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
  # mc_rtc.mc_tasks submodule
  src/mc_rtc_module/mc_tasks/mc_tasks_module.cpp
  src/mc_rtc_module/mc_tasks/bind_MetaTask.cpp
  src/mc_rtc_module/mc_tasks/bind_TrajectoryTaskGeneric.cpp
  src/mc_rtc_module/mc_tasks/bind_PostureTask.cpp
  src/mc_rtc_module/mc_tasks/bind_TransformTask.cpp
  src/mc_rtc_module/mc_tasks/bind_GazeTask.cpp
  src/mc_rtc_module/mc_tasks/bind_PositionTask.cpp
  src/mc_rtc_module/mc_tasks/bind_OrientationTask.cpp
  src/mc_rtc_module/mc_tasks/bind_EndEffectorTask.cpp
  src/mc_rtc_module/mc_tasks/bind_ImpedanceTask.cpp
  src/mc_rtc_module/mc_tasks/bind_CoMTask.cpp
  src/mc_rtc_module/mc_tasks/bind_CoPTask.cpp
  src/mc_rtc_module/mc_tasks/bind_ComplianceTask.cpp
  src/mc_rtc_module/mc_tasks/bind_BSplineTrajectoryTask.cpp
  # mc_rtc.mc_solver submodule
  src/mc_rtc_module/mc_solver/mc_solver_module.cpp
  src/mc_rtc_module/mc_solver/bind_QPSolver.cpp
  src/mc_rtc_module/mc_solver/bind_KinematicsConstraint.cpp
  src/mc_rtc_module/mc_solver/bind_DynamicsConstraint.cpp
  src/mc_rtc_module/mc_solver/bind_ContactConstraint.cpp
  src/mc_rtc_module/mc_solver/bind_CompoundJointConstraint.cpp
  src/mc_rtc_module/mc_solver/bind_CoMIncPlaneConstr.cpp
  src/mc_rtc_module/mc_solver/bind_CollisionsConstraint.cpp
  src/mc_rtc_module/mc_solver/bind_BoundedSpeedConstr.cpp
)

jrl_check_python_module_name(_mc_rtc)
target_link_libraries(_mc_rtc PRIVATE mc_rtc::mc_rtc_utils mc_rtc::mc_rbdyn mc_rtc::mc_tasks mc_rtc::mc_solver mc_rtc::mc_control)
target_include_directories(
  _mc_rtc PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
jrl_target_set_default_compile_options(_mc_rtc PRIVATE)
jrl_target_set_output_directory(_mc_rtc OUTPUT_DIRECTORY
                                ${CMAKE_BINARY_DIR}/lib/site-packages/mc_rtc)


set(MC_RTC_MODULES mc_rtc mc_rbdyn mc_control mc_tasks mc_solver)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
               ${CMAKE_BINARY_DIR}/lib/site-packages/mc_rtc/__init__.py COPYONLY)

foreach(MODULE IN LISTS MC_RTC_MODULES)
    set(MOD_DIR "${CMAKE_BINARY_DIR}/lib/site-packages/mc_rtc/${MODULE}")
    file(MAKE_DIRECTORY "${MOD_DIR}")

    set(MODULE_NAME ${MODULE})
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/__init__stub.conf.py
        ${MOD_DIR}/__init__.py
        @ONLY
    )

    nanobind_add_stub(
        mc_rtc_${MODULE}_stub
        MODULE
        mc_rtc._mc_rtc.${MODULE}
        OUTPUT
        ${MOD_DIR}/__init__.pyi
        MARKER_FILE
        ${MOD_DIR}/py.typed
        PYTHON_PATH
        ${CMAKE_BINARY_DIR}/lib/site-packages
        DEPENDS
        _mc_rtc
        VERBOSE
    )
endforeach()

jrl_python_compile_all(DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/mc_rtc
                       VERBOSE)

jrl_python_compute_install_dir(python_install_dir)
install(
  DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/mc_rtc
  DESTINATION ${python_install_dir}
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "*.pyc"
  PATTERN "*.pyi"
  PATTERN "*.typed"
  PATTERN "*.so"
  PATTERN "*.pyd")
