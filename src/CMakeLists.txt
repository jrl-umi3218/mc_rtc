#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(MC_RTC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include")

set(VERSION_SCRIPT_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/version-script.lds")
macro(install_mc_rtc_lib MC_LIB)
  set_target_properties(${MC_LIB} PROPERTIES FOLDER mc_libraries/${MC_LIB})
  target_include_directories(${MC_LIB} INTERFACE $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${MC_RTC_INCLUDE_DIR}>)
  if(WIN32)
    target_compile_definitions(${MC_LIB} PUBLIC "-DNOMINMAX"  "-D_ENABLE_EXTENDED_ALIGNED_STORAGE")
  else()
    target_compile_options(${MC_LIB} PUBLIC "-mfpmath=sse" "-msse2")
  endif()
  if(TREAT_WARNINGS_AS_ERRORS)
    if(WIN32)
      target_compile_options(${MC_LIB} PUBLIC $<BUILD_INTERFACE:/WX>)
    else()
      target_compile_options(${MC_LIB} PUBLIC $<BUILD_INTERFACE:-Werror>)
    endif()
  endif()
  if(GENERATE_COVERAGE)
    target_compile_options(${MC_LIB} PRIVATE "--coverage")
    set_target_properties(${MC_LIB} PROPERTIES LINK_FLAGS "--coverage")
  endif()
  set_target_properties(${MC_LIB} PROPERTIES SOVERSION 1 VERSION 1.0.0)
  add_version_script(${MC_LIB} "${VERSION_SCRIPT_LOCATION}")
  install(TARGETS ${MC_LIB}
    EXPORT "${TARGETS_EXPORT_NAME}"
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endmacro()

configure_file(mc_rtc/loader.in.cpp "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc/loader.cpp")
add_library(mc_rtc_loader SHARED "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc/loader.cpp" ../include/mc_rtc/loader.h ../include/mc_rtc/loader_api.h)
set_target_properties(mc_rtc_loader PROPERTIES COMPILE_FLAGS "-DMC_RTC_LOADER_EXPORTS")
target_link_libraries(mc_rtc_loader PUBLIC Boost::filesystem Boost::disable_autolinking ${LTDL})
install_mc_rtc_lib(mc_rtc_loader)

set(mc_rtc_utils_SRC
  mc_rtc/Configuration.cpp
  mc_rtc/FlatLog.cpp
  mc_rtc/iterate_binary_log.cpp
  mc_rtc/Logger.cpp
  mc_rtc/MessagePackBuilder.cpp
)

set(mc_rtc_utils_HDR
  ../include/mc_rtc/Configuration.h
  ../include/mc_rtc/MessagePackBuilder.h
  ../include/mc_rtc/log/FlatLog.h
  ../include/mc_rtc/log/iterate_binary_log.h
  ../include/mc_rtc/log/Logger.h
  ../include/mc_rtc/utils.h
  ../include/mc_rtc/utils_api.h
)

add_library(mc_rtc_utils SHARED ${mc_rtc_utils_SRC} ${mc_rtc_utils_HDR})
set_target_properties(mc_rtc_utils PROPERTIES COMPILE_FLAGS "-DMC_RTC_UTILS_EXPORTS")
target_link_libraries(mc_rtc_utils PUBLIC SpaceVecAlg::SpaceVecAlg Boost::filesystem Boost::disable_autolinking PRIVATE mc_rtc_3rd_party::RapidJSON mpack)
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
)

add_library(mc_rbdyn SHARED ${mc_rbdyn_SRC} ${mc_rbdyn_HDR})
set_target_properties(mc_rbdyn PROPERTIES COMPILE_FLAGS "-DMC_RBDYN_EXPORTS")
target_link_libraries(mc_rbdyn PUBLIC mc_rbdyn_urdf::mc_rbdyn_urdf Tasks::Tasks mc_rtc_loader mc_rtc_utils GEOS::geos)
# qhull
target_include_directories(mc_rbdyn SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../3rd-party/qhull/src")
target_link_libraries(mc_rbdyn PRIVATE qhullcpp qhullstatic_r)
install_mc_rtc_lib(mc_rbdyn)

set(mc_rtc_gui_SRC
  mc_rtc/GUIState.cpp
)

set(mc_rtc_gui_HDR
  ../include/mc_rtc/GUIDetails.h
  ../include/mc_rtc/GUIState.h
  ../include/mc_rtc/GUIState.hpp
  ../include/mc_rtc/GUITypes.h
  ../include/mc_rtc/gui_api.h
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
mc_solver/DynamicsConstraint.cpp
mc_solver/KinematicsConstraint.cpp
mc_solver/QPSolver.cpp
mc_tasks/MetaTask.cpp
)

set(mc_solver_HDR
../include/mc_solver/api.h
../include/mc_solver/msg/Contact.h
../include/mc_solver/msg/QPResult.h
../include/mc_solver/msg/Robot.h
../include/mc_solver/BoundedSpeedConstr.h
../include/mc_solver/CollisionsConstraint.h
../include/mc_solver/CoMIncPlaneConstr.h
../include/mc_solver/CompoundJointConstraint.h
../include/mc_solver/ConstraintSet.h
../include/mc_solver/ContactConstraint.h
../include/mc_solver/DynamicsConstraint.h
../include/mc_solver/KinematicsConstraint.h
../include/mc_solver/QPSolver.h
../include/mc_solver/GenericLoader.h
../include/mc_solver/GenericLoader.hpp
../include/mc_tasks/MetaTask.h
)

add_library(mc_solver SHARED ${mc_solver_SRC} ${mc_solver_HDR})
set_target_properties(mc_solver PROPERTIES COMPILE_FLAGS "-DMC_SOLVER_EXPORTS")
target_link_libraries(mc_solver PUBLIC mc_rbdyn mc_rtc_gui)
install_mc_rtc_lib(mc_solver)

set(mc_trajectory_SRC
mc_trajectory/BSplineTrajectory.cpp
mc_trajectory/spline_utils.cpp
)

set(mc_trajectory_HDR
../include/mc_trajectory/api.h
../include/mc_trajectory/BSplineTrajectory.h
../include/mc_trajectory/spline_utils.h
)

add_library(mc_trajectory SHARED ${mc_trajectory_SRC} ${mc_trajectory_HDR})
set_target_properties(mc_trajectory PROPERTIES COMPILE_FLAGS "-DMC_TRAJECTORY_EXPORTS")
target_link_libraries(mc_trajectory PUBLIC SpaceVecAlg::SpaceVecAlg)
install_mc_rtc_lib(mc_trajectory)

set(mc_tasks_SRC
mc_tasks/PositionTask.cpp
mc_tasks/SurfaceTransformTask.cpp
mc_tasks/OrientationTask.cpp
mc_tasks/VectorOrientationTask.cpp
mc_tasks/EndEffectorTask.cpp
mc_tasks/RelativeEndEffectorTask.cpp
mc_tasks/CoMTask.cpp
mc_tasks/AddRemoveContactTask.cpp
mc_tasks/TrajectoryTask.cpp
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
)

set(mc_tasks_HDR
../include/mc_tasks/AddRemoveContactTask.h
../include/mc_tasks/api.h
../include/mc_tasks/CoMTask.h
../include/mc_tasks/EndEffectorTask.h
../include/mc_tasks/MetaTaskLoader.h
../include/mc_tasks/OrientationTask.h
../include/mc_tasks/VectorOrientationTask.h
../include/mc_tasks/PositionTask.h
../include/mc_tasks/SurfaceTransformTask.h
../include/mc_tasks/RelativeEndEffectorTask.h
../include/mc_tasks/SmoothTask.h
../include/mc_tasks/TrajectoryTask.h
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
)

if(NOT ${DISABLE_ROS})
  if(${ROSCPP_FOUND})
    set(mc_tasks_SRC
      ${mc_tasks_SRC}
      mc_tasks/LookAtTFTask.cpp
      )
    set(mc_tasks_HDR
      ${mc_tasks_HDR}
      ../include/mc_tasks/LookAtTFTask.h
      )
  endif()
endif()

add_library(mc_tasks SHARED ${mc_tasks_SRC} ${mc_tasks_HDR})
set_target_properties(mc_tasks PROPERTIES COMPILE_FLAGS "-DMC_TASKS_EXPORTS")
target_link_libraries(mc_tasks PUBLIC mc_solver mc_rbdyn mc_trajectory mc_rtc_utils)
if(NOT ${DISABLE_ROS})
  if(${ROSCPP_FOUND})
    target_link_libraries(mc_tasks PUBLIC mc_rtc_3rd_party::ROS)
  endif()
endif()
install_mc_rtc_lib(mc_tasks)


set(mc_observers_SRC
mc_observers/ObserverLoader.cpp
mc_observers/Observer.cpp
)

set(mc_observers_HDR
../include/mc_observers/api.h
../include/mc_observers/ObserverLoader.h
../include/mc_observers/Observer.h
../include/mc_observers/LowPassVelocityFilter.h
../include/mc_observers/LowPassFiniteDifferencesVelocityFilter.h
)

add_library(mc_observers SHARED ${mc_observers_SRC} ${mc_observers_HDR})
set_target_properties(mc_observers PROPERTIES COMPILE_FLAGS "-DMC_OBSERVERS_EXPORTS")
target_link_libraries(mc_observers PUBLIC mc_rbdyn mc_rtc_gui)
install_mc_rtc_lib(mc_observers)


add_library(mc_rtc_ros SHARED mc_rtc/ros.cpp ../include/mc_rtc/ros.h ../include/mc_rtc/ros_api.h)
set_target_properties(mc_rtc_ros PROPERTIES COMPILE_FLAGS "-DMC_RTC_ROS_EXPORTS")
target_link_libraries(mc_rtc_ros PUBLIC mc_rbdyn)
if(NOT ${DISABLE_ROS})
  if(${ROSCPP_FOUND})
    target_link_libraries(mc_rtc_ros PUBLIC mc_rtc_3rd_party::ROS nanomsg)
  endif()
endif()
install_mc_rtc_lib(mc_rtc_ros)

set(mc_control_SRC
mc_control/CompletionCriteria.cpp
mc_control/ControllerServer.cpp
mc_control/SimulationContactPair.cpp
mc_control/generic_gripper.cpp
mc_control/mc_controller.cpp
mc_control/mc_python_controller.cpp
mc_control/mc_global_controller.cpp
mc_control/mc_global_controller_configuration.cpp
mc_control/mc_global_controller_gui.cpp
mc_control/mc_global_controller_services.cpp
mc_control/mc_global_controller_ros_services.h
mc_control/mc_global_controller_ros_services.cpp
)

set(mc_control_HDR
../include/mc_control/api.h
../include/mc_control/CompletionCriteria.h
../include/mc_control/Configuration.h
../include/mc_control/ControllerServer.h
../include/mc_control/generic_gripper.h
../include/mc_control/mc_controller.h
../include/mc_control/mc_python_controller.h
../include/mc_control/SimulationContactPair.h
../include/mc_control/mc_global_controller.h
)

add_library(mc_control SHARED ${mc_control_SRC} ${mc_control_HDR})
set_target_properties(mc_control PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
target_link_libraries(mc_control PUBLIC mc_tasks mc_solver mc_rtc_ros mc_rtc_loader mc_observers mc_rtc_utils mc_rtc_gui nanomsg)
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
