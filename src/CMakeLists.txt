#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(MC_RTC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include")

set(VERSION_SCRIPT_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/version-script.lds")

macro(install_mc_rtc_lib_common MC_LIB KWD)
  target_include_directories(${MC_LIB} INTERFACE $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${MC_RTC_INCLUDE_DIR}>)
  if(WIN32)
    target_compile_definitions(${MC_LIB} ${KWD} "-DNOMINMAX"  "-D_ENABLE_EXTENDED_ALIGNED_STORAGE")
  else()
    target_compile_options(${MC_LIB} ${KWD} "-mfpmath=sse" "-msse2")
  endif()
  if(TREAT_WARNINGS_AS_ERRORS)
    if(WIN32)
      target_compile_options(${MC_LIB} ${KWD} $<BUILD_INTERFACE:/WX>)
    else()
      target_compile_options(${MC_LIB} ${KWD} $<BUILD_INTERFACE:-Werror>)
    endif()
  endif()
  add_library(mc_rtc::${MC_LIB} ALIAS ${MC_LIB})
endmacro()

macro(install_mc_rtc_header_lib MC_LIB)
  install_mc_rtc_lib_common(${MC_LIB} INTERFACE)
  install(TARGETS ${MC_LIB}
    EXPORT "${TARGETS_EXPORT_NAME}"
  )
endmacro()

macro(install_mc_rtc_lib MC_LIB)
  set_target_properties(${MC_LIB} PROPERTIES FOLDER mc_libraries/${MC_LIB})
  install_mc_rtc_lib_common(${MC_LIB} PUBLIC)
  if(GENERATE_COVERAGE)
    target_compile_options(${MC_LIB} PRIVATE "--coverage")
    set_target_properties(${MC_LIB} PROPERTIES LINK_FLAGS "--coverage")
  endif()
  set_target_properties(${MC_LIB} PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
  add_version_script(${MC_LIB} "${VERSION_SCRIPT_LOCATION}")
  add_library(mc_rtc::${MC_LIB} ALIAS ${MC_LIB})
  install(TARGETS ${MC_LIB}
    EXPORT "${TARGETS_EXPORT_NAME}"
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endmacro()

configure_file(mc_rtc/loader.in.cpp "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc/loader.cpp")

set(VERSION_HEADER "${PROJECT_BINARY_DIR}/include/mc_rtc/version.h")
add_custom_command(OUTPUT ${VERSION_HEADER}
  COMMAND ${CMAKE_COMMAND} -DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
                           -DPROJECT_BINARY_DIR="${PROJECT_BINARY_DIR}"
                           -DPROJECT_VERSION="${PROJECT_VERSION}"
                           -P "${PROJECT_SOURCE_DIR}/CMakeModules/MakeVersionHeader.cmake"
  DEPENDS ${MC_RTC_ALL_FILES}
)

set(mc_rtc_loader_SRC
  "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc/loader.cpp"
  mc_rtc/version.cpp
)

set(mc_rtc_loader_HDR
  "${VERSION_HEADER}"
  ../include/mc_rtc/loader.h
  ../include/mc_rtc/loader_api.h
)

add_library(mc_rtc_loader SHARED ${mc_rtc_loader_SRC} ${mc_rtc_loader_HDR})
set_target_properties(mc_rtc_loader PROPERTIES COMPILE_FLAGS "-DMC_RTC_LOADER_EXPORTS")
target_include_directories(mc_rtc_loader PRIVATE "${PROJECT_BINARY_DIR}/include/mc_rtc")
target_link_libraries(mc_rtc_loader PUBLIC Boost::filesystem Boost::disable_autolinking ${LTDL})
install_mc_rtc_lib(mc_rtc_loader)

set(mc_rtc_utils_SRC
  mc_rtc/Configuration.cpp
  mc_rtc/DataStore.cpp
  mc_rtc/FlatLog.cpp
  mc_rtc/iterate_binary_log.cpp
  mc_rtc/Logger.cpp
  mc_rtc/MessagePackBuilder.cpp
)

set(mc_rtc_utils_HDR
  mc_rtc/internals/json.h
  mc_rtc/internals/yaml.h
  mc_rtc/internals/LogEntry.h
  ../include/mc_rtc/Configuration.h
  ../include/mc_rtc/ConfigurationHelpers.h
  ../include/mc_rtc/MessagePackBuilder.h
  ../include/mc_rtc/log/FlatLog.h
  ../include/mc_rtc/log/iterate_binary_log.h
  ../include/mc_rtc/log/Logger.h
  ../include/mc_rtc/io_utils.h
  ../include/mc_rtc/utils.h
  ../include/mc_rtc/utils_api.h
  ../include/mc_rtc/constants.h
  ../include/mc_rtc/DataStore.h
  ../include/mc_rtc/type_name.h
)

add_library(mc_rtc_utils SHARED ${mc_rtc_utils_SRC} ${mc_rtc_utils_HDR})
set_target_properties(mc_rtc_utils PROPERTIES COMPILE_FLAGS "-DMC_RTC_UTILS_EXPORTS")
target_link_libraries(mc_rtc_utils PUBLIC SpaceVecAlg::SpaceVecAlg Boost::filesystem Boost::disable_autolinking yaml-cpp PRIVATE mc_rtc_3rd_party::RapidJSON mpack)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_link_libraries(mc_rtc_utils PUBLIC atomic)
endif()
install_mc_rtc_lib(mc_rtc_utils)

set(mc_rbdyn_SRC
mc_rbdyn/SCHAddon.cpp
mc_rbdyn/contact_transform.cpp
mc_rbdyn/Surface.cpp
mc_rbdyn/PlanarSurface.cpp
mc_rbdyn/CylindricalSurface.cpp
mc_rbdyn/GripperSurface.cpp
mc_rbdyn/surface_utils.cpp
mc_rbdyn/surface_hull.cpp
mc_rbdyn/Contact.cpp
mc_rbdyn/Robot.cpp
mc_rbdyn/Robots.cpp
mc_rbdyn/PolygonInterpolator.cpp
mc_rbdyn/polygon_utils.cpp
mc_rbdyn/RobotLoader.cpp
mc_rbdyn/Collision.cpp
mc_rbdyn/ForceSensor.cpp
mc_rbdyn/RobotModule.cpp
mc_rbdyn/configuration_io.cpp
mc_rbdyn/ZMP.cpp
mc_control/generic_gripper.cpp
mc_rbdyn/Device.cpp
mc_rbdyn/BodySensor.cpp
)

set(mc_rbdyn_HDR
../include/mc_rbdyn/api.h
../include/mc_rbdyn/Contact.h
../include/mc_rbdyn/contact_transform.h
../include/mc_rbdyn/CylindricalSurface.h
../include/mc_rbdyn/GripperSurface.h
../include/mc_rbdyn/Mimic.h
../include/mc_rbdyn/PlanarSurface.h
../include/mc_rbdyn/polygon_utils.h
../include/mc_rbdyn/PolygonInterpolator.h
../include/mc_rbdyn/Robot.h
../include/mc_rbdyn/Robots.h
../include/mc_rbdyn/RobotLoader.h
../include/mc_rbdyn/RobotModule.h
../include/mc_rbdyn/RobotModuleMacros.h
../include/mc_rbdyn/SCHAddon.h
../include/mc_rbdyn/Surface.h
../include/mc_rbdyn/rpy_utils.h
../include/mc_rbdyn/surface_hull.h
../include/mc_rbdyn/surface_utils.h
../include/mc_rbdyn/Collision.h
../include/mc_rbdyn/CompoundJointConstraintDescription.h
../include/mc_rbdyn/ForceSensor.h
../include/mc_rbdyn/Flexibility.h
../include/mc_rbdyn/Springs.h
../include/mc_rbdyn/Base.h
../include/mc_rbdyn/BodySensor.h
../include/mc_rbdyn/configuration_io.h
../include/mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h
../include/mc_rbdyn/lipm_stabilizer/ZMPCCConfiguration.h
../include/mc_rbdyn/ZMP.h
../include/mc_control/generic_gripper.h
../include/mc_rbdyn/Device.h
)

add_library(mc_rbdyn SHARED ${mc_rbdyn_SRC} ${mc_rbdyn_HDR})
set_target_properties(mc_rbdyn PROPERTIES COMPILE_FLAGS "-DMC_RBDYN_EXPORTS")
target_link_libraries(mc_rbdyn PUBLIC mc_rbdyn_urdf::mc_rbdyn_urdf RBDyn::Parsers Tasks::Tasks mc_rtc_loader mc_rtc_utils GEOS::geos)
# qhull
target_include_directories(mc_rbdyn SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../3rd-party/qhull/src")
target_link_libraries(mc_rbdyn PRIVATE qhullcpp qhullstatic_r)
install_mc_rtc_lib(mc_rbdyn)

set(mc_rtc_gui_SRC
  mc_rtc/gui/StateBuilder.cpp
)

set(mc_rtc_gui_HDR
  ../include/mc_rtc/gui/api.h
  ../include/mc_rtc/gui/StateBuilder.h
  ../include/mc_rtc/gui/StateBuilder.hpp
  ../include/mc_rtc/gui/ArrayInput.h
  ../include/mc_rtc/gui/ArrayLabel.h
  ../include/mc_rtc/gui/Arrow.h
  ../include/mc_rtc/gui/Button.h
  ../include/mc_rtc/gui/Checkbox.h
  ../include/mc_rtc/gui/ComboInput.h
  ../include/mc_rtc/gui/DataComboInput.h
  ../include/mc_rtc/gui/elements.h
  ../include/mc_rtc/gui/Force.h
  ../include/mc_rtc/gui/Form.h
  ../include/mc_rtc/gui/IntegerInput.h
  ../include/mc_rtc/gui/Label.h
  ../include/mc_rtc/gui/NumberInput.h
  ../include/mc_rtc/gui/NumberSlider.h
  ../include/mc_rtc/gui/Point3D.h
  ../include/mc_rtc/gui/Polygon.h
  ../include/mc_rtc/gui/Rotation.h
  ../include/mc_rtc/gui/Schema.h
  ../include/mc_rtc/gui/StringInput.h
  ../include/mc_rtc/gui/Trajectory.h
  ../include/mc_rtc/gui/Transform.h
  ../include/mc_rtc/gui/types.h
  ../include/mc_rtc/gui/XYTheta.h
  ../include/mc_rtc/gui/details/traits.h

  ../include/mc_rtc/gui/plot.h
  ../include/mc_rtc/gui/plot/Abscissa.h
  ../include/mc_rtc/gui/plot/AbscissaOrdinate.h
  ../include/mc_rtc/gui/plot/types.h
  ../include/mc_rtc/gui/plot/Ordinate.h
  ../include/mc_rtc/gui/plot/Polygon.h
  ../include/mc_rtc/gui/plot/Polygons.h
)

add_library(mc_rtc_gui SHARED ${mc_rtc_gui_SRC} ${mc_rtc_gui_HDR})
set_target_properties(mc_rtc_gui PROPERTIES COMPILE_FLAGS "-DMC_RTC_GUI_EXPORTS")
target_link_libraries(mc_rtc_gui PUBLIC mc_rbdyn)
install_mc_rtc_lib(mc_rtc_gui)

add_subdirectory(mc_robots)
add_subdirectory(mc_observers)

set(mc_solver_SRC
mc_solver/BoundedSpeedConstr.cpp
mc_solver/CollisionsConstraint.cpp
mc_solver/CoMIncPlaneConstr.cpp
mc_solver/CompoundJointConstraint.cpp
mc_solver/ConstraintSetLoader.cpp
mc_solver/ContactConstraint.cpp
mc_solver/ContactWrenchMatrixToLambdaMatrix.cpp
mc_solver/DynamicsConstraint.cpp
mc_solver/KinematicsConstraint.cpp
mc_solver/QPSolver.cpp
mc_solver/Update.cpp
mc_solver/UpdateNrVars.cpp
mc_tasks/MetaTask.cpp
)

set(mc_solver_HDR
../include/mc_solver/api.h
../include/mc_solver/msg/Contact.h
../include/mc_solver/msg/QPResult.h
../include/mc_solver/msg/Robot.h
../include/mc_solver/utils/Constraint.h
../include/mc_solver/utils/ContactWrenchMatrixToLambdaMatrix.h
../include/mc_solver/utils/Update.h
../include/mc_solver/utils/UpdateNrVars.h
../include/mc_solver/BoundedSpeedConstr.h
../include/mc_solver/CollisionsConstraint.h
../include/mc_solver/CoMIncPlaneConstr.h
../include/mc_solver/CompoundJointConstraint.h
../include/mc_solver/ConstraintSet.h
../include/mc_solver/ContactConstraint.h
../include/mc_solver/DynamicsConstraint.h
../include/mc_solver/EqualityConstraint.h
../include/mc_solver/GenInequalityConstraint.h
../include/mc_solver/InequalityConstraint.h
../include/mc_solver/KinematicsConstraint.h
../include/mc_solver/GenericLoader.h
../include/mc_solver/GenericLoader.hpp
../include/mc_solver/GenInequalityConstraint.h
../include/mc_solver/InequalityConstraint.h
../include/mc_solver/QPSolver.h
../include/mc_tasks/MetaTask.h
)

add_library(mc_solver SHARED ${mc_solver_SRC} ${mc_solver_HDR})
set_target_properties(mc_solver PROPERTIES COMPILE_FLAGS "-DMC_SOLVER_EXPORTS")
target_link_libraries(mc_solver PUBLIC mc_rbdyn mc_rtc_gui)
install_mc_rtc_lib(mc_solver)

set(mc_trajectory_SRC
mc_trajectory/BSpline.cpp
mc_trajectory/InterpolatedRotation.cpp
mc_trajectory/spline_utils.cpp
mc_trajectory/ExactCubic.cpp
)

set(mc_trajectory_HDR
../include/mc_trajectory/api.h
../include/mc_trajectory/Spline.h
../include/mc_trajectory/Spline.hpp
../include/mc_trajectory/BSpline.h
../include/mc_trajectory/InterpolatedRotation.h
../include/mc_trajectory/spline_utils.h
../include/mc_trajectory/ExactCubic.h
)

add_library(mc_trajectory SHARED ${mc_trajectory_SRC} ${mc_trajectory_HDR})
set_target_properties(mc_trajectory PROPERTIES COMPILE_FLAGS "-DMC_TRAJECTORY_EXPORTS")
target_link_libraries(mc_trajectory PUBLIC mc_rtc_3rd_party::hpp-spline mc_rtc::mc_rtc_gui)
install_mc_rtc_lib(mc_trajectory)

set(mc_tasks_SRC
mc_tasks/MomentumTask.cpp
mc_tasks/PositionTask.cpp
mc_tasks/SurfaceTransformTask.cpp
mc_tasks/OrientationTask.cpp
mc_tasks/VectorOrientationTask.cpp
mc_tasks/EndEffectorTask.cpp
mc_tasks/RelativeEndEffectorTask.cpp
mc_tasks/CoMTask.cpp
mc_tasks/AddRemoveContactTask.cpp
mc_tasks/BSplineTrajectoryTask.cpp
mc_tasks/ExactCubicTrajectoryTask.cpp
mc_tasks/ComplianceTask.cpp
mc_tasks/AdmittanceTask.cpp
mc_tasks/DampingTask.cpp
mc_tasks/CoPTask.cpp
mc_tasks/GazeTask.cpp
mc_tasks/PositionBasedVisServoTask.cpp
mc_tasks/PostureTask.cpp
mc_tasks/LookAtTask.cpp
mc_tasks/LookAtSurfaceTask.cpp
mc_tasks/MetaTaskLoader.cpp
mc_tasks/lipm_stabilizer/StabilizerTask.cpp
mc_tasks/lipm_stabilizer/StabilizerTask_log_gui.cpp
mc_tasks/lipm_stabilizer/ZMPCC.cpp
mc_tasks/lipm_stabilizer/Contact.cpp
)

set(mc_tasks_HDR
../include/mc_tasks/AddRemoveContactTask.h
../include/mc_tasks/api.h
../include/mc_tasks/CoMTask.h
../include/mc_tasks/EndEffectorTask.h
../include/mc_tasks/MetaTaskLoader.h
../include/mc_tasks/MomentumTask.h
../include/mc_tasks/OrientationTask.h
../include/mc_tasks/VectorOrientationTask.h
../include/mc_tasks/PositionTask.h
../include/mc_tasks/SurfaceTransformTask.h
../include/mc_tasks/RelativeEndEffectorTask.h
../include/mc_tasks/SmoothTask.h
../include/mc_tasks/SplineTrajectoryTask.h
../include/mc_tasks/SplineTrajectoryTask.hpp
../include/mc_tasks/BSplineTrajectoryTask.h
../include/mc_tasks/ExactCubicTrajectoryTask.h
../include/mc_tasks/TrajectoryTaskGeneric.h
../include/mc_tasks/TrajectoryTaskGeneric.hpp
../include/mc_tasks/ComplianceTask.h
../include/mc_tasks/AdmittanceTask.h
../include/mc_tasks/DampingTask.h
../include/mc_tasks/CoPTask.h
../include/mc_tasks/GazeTask.h
../include/mc_tasks/PositionBasedVisServoTask.h
../include/mc_tasks/PostureTask.h
../include/mc_tasks/LookAtTask.h
../include/mc_tasks/ExactCubicTrajectoryTask.h
../include/mc_tasks/lipm_stabilizer/StabilizerTask.h
../include/mc_tasks/lipm_stabilizer/Contact.h
../include/mc_tasks/lipm_stabilizer/ZMPCC.h
)

add_library(mc_tasks SHARED ${mc_tasks_SRC} ${mc_tasks_HDR})
set_target_properties(mc_tasks PROPERTIES COMPILE_FLAGS "-DMC_TASKS_EXPORTS")
target_link_libraries(mc_tasks PUBLIC mc_solver mc_rbdyn mc_trajectory mc_filter mc_rtc_utils eigen-quadprog::eigen-quadprog)
install_mc_rtc_lib(mc_tasks)

set(mc_planning_SRC
mc_planning/Pendulum.cpp
)

set(mc_planning_HDR
../include/mc_planning/Pendulum.h
../include/mc_planning/api.h
)

add_library(mc_planning SHARED ${mc_planning_SRC} ${mc_planning_HDR})
set_target_properties(mc_planning PROPERTIES COMPILE_FLAGS "-DMC_PLANNING_EXPORTS")
target_link_libraries(mc_planning PUBLIC mc_rbdyn)
install_mc_rtc_lib(mc_planning)

set(mc_filter_HDR_DIR
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/mc_filter>$<INSTALL_INTERFACE:include/mc_filter>)

set(mc_filter_HDR
  ${mc_filter_HDR_DIR}/LowPass.h
  ${mc_filter_HDR_DIR}/LowPassFiniteDifferences.h
  ${mc_filter_HDR_DIR}/ExponentialMovingAverage.h
  ${mc_filter_HDR_DIR}/LeakyIntegrator.h
  ${mc_filter_HDR_DIR}/StationaryOffset.h
  ${mc_filter_HDR_DIR}/utils/clamp.h
)

# header-only
add_library(mc_filter INTERFACE)
if(COMMAND target_sources)
  target_sources(mc_filter INTERFACE ${mc_filter_HDR})
endif()
install_mc_rtc_header_lib(mc_filter)

set(mc_observers_SRC
mc_observers/ObserverLoader.cpp
mc_observers/Observer.cpp
)

set(mc_observers_HDR
../include/mc_observers/api.h
../include/mc_observers/ObserverLoader.h
../include/mc_observers/Observer.h
../include/mc_observers/ObserverMacros.h
)

add_library(mc_observers SHARED ${mc_observers_SRC} ${mc_observers_HDR})
set_target_properties(mc_observers PROPERTIES COMPILE_FLAGS "-DMC_OBSERVERS_EXPORTS")
target_link_libraries(mc_observers PUBLIC mc_filter mc_rbdyn mc_rtc_gui)
install_mc_rtc_lib(mc_observers)

set(mc_control_SRC
mc_control/CompletionCriteria.cpp
mc_control/ControllerServer.cpp
mc_control/SimulationContactPair.cpp
mc_control/MCController.cpp
mc_control/mc_python_controller.cpp
mc_control/mc_global_controller.cpp
mc_control/mc_global_controller_configuration.cpp
mc_control/mc_global_controller_gui.cpp
mc_control/mc_global_controller_services.cpp
)

set(mc_control_HDR
../include/mc_control/api.h
../include/mc_control/CompletionCriteria.h
../include/mc_control/Configuration.h
../include/mc_control/ControllerServer.h
../include/mc_control/GlobalPlugin.h
../include/mc_control/GlobalPluginMacros.h
../include/mc_control/GlobalPlugin_fwd.h
../include/mc_control/MCController.h
../include/mc_control/mc_controller.h
../include/mc_control/mc_python_controller.h
../include/mc_control/SimulationContactPair.h
../include/mc_control/mc_global_controller.h
)

add_library(mc_control SHARED ${mc_control_SRC} ${mc_control_HDR})
set_target_properties(mc_control PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
target_link_libraries(mc_control PUBLIC mc_tasks mc_solver mc_rtc_loader mc_observers mc_rtc_utils mc_rtc_gui nanomsg)
install_mc_rtc_lib(mc_control)

set(mc_control_client_SRC
  mc_control/ControllerClient.cpp
)

set(mc_control_client_HDR
  ../include/mc_control/client_api.h
  ../include/mc_control/ControllerClient.h
)

add_library(mc_control_client SHARED ${mc_control_client_SRC} ${mc_control_client_HDR})
set_target_properties(mc_control_client PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_CLIENT_EXPORTS")
target_link_libraries(mc_control_client PUBLIC mc_control)
install_mc_rtc_lib(mc_control_client)

add_subdirectory(mc_control)
