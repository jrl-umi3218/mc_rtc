link_directories(${Boost_LIBRARY_DIR_RELEASE})

if(WIN32)
  add_definitions( -DBOOST_ALL_DYN_LINK )
endif()

add_library(mc_rtc_loader SHARED mc_rtc/loader.cpp ../include/mc_rtc/loader.h ../include/mc_rtc/loader_api.h)
set_target_properties(mc_rtc_loader PROPERTIES COMPILE_FLAGS "-DMC_RTC_LOADER_EXPORTS")
target_link_libraries(mc_rtc_loader ${Boost_LIBRARIES} ltdl)
pkg_config_append_libs(mc_rtc_loader)

set(mc_rtc_utils_SRC
  mc_rtc/Configuration.cpp
  mc_rtc/Logger.cpp
)
set(mc_rtc_utils_SRC ${mc_rtc_utils_SRC} PARENT_SCOPE)

set(mc_rtc_utils_HDR
  ../include/mc_rtc/Configuration.h
  ../include/mc_rtc/log/Logger.h
  ../include/mc_rtc/log/serialization/MCLog_generated.h
  ../include/mc_rtc/utils.h
  ../include/mc_rtc/utils_api.h
)
set(mc_rtc_utils_HDR ${mc_rtc_utils_HDR} PARENT_SCOPE)

add_library(mc_rtc_utils SHARED ${mc_rtc_utils_SRC} ${mc_rtc_utils_HDR})
set_target_properties(mc_rtc_utils PROPERTIES COMPILE_FLAGS "-DMC_RTC_UTILS_EXPORTS")
target_link_libraries(mc_rtc_utils ${Boost_LIBRARIES})
pkg_config_append_libs(mc_rtc_utils)

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
mc_rbdyn/stance.cpp
mc_rbdyn/stance.cpp
mc_rbdyn/StanceConfig.cpp
mc_rbdyn/PolygonInterpolator.cpp
mc_rbdyn/polygon_utils.cpp
mc_rbdyn/RobotLoader.cpp
mc_rbdyn/Collision.cpp
mc_rbdyn/ForceSensor.cpp
mc_rbdyn/RobotModule.cpp
mc_rbdyn/configuration_io.cpp
)
set(mc_rbdyn_SRC ${mc_rbdyn_SRC} PARENT_SCOPE)

set(mc_rbdyn_HDR
../include/mc_rbdyn/api.h
../include/mc_rbdyn/Contact.h
../include/mc_rbdyn/contact_transform.h
../include/mc_rbdyn/CylindricalSurface.h
../include/mc_rbdyn/GripperSurface.h
../include/mc_rbdyn/PlanarSurface.h
../include/mc_rbdyn/polygon_utils.h
../include/mc_rbdyn/PolygonInterpolator.h
../include/mc_rbdyn/Robot.h
../include/mc_rbdyn/Robots.h
../include/mc_rbdyn/RobotLoader.h
../include/mc_rbdyn/RobotModule.h
../include/mc_rbdyn/SCHAddon.h
../include/mc_rbdyn/stance.h
../include/mc_rbdyn/StanceConfig.h
../include/mc_rbdyn/Surface.h
../include/mc_rbdyn/rpy_utils.h
../include/mc_rbdyn/surface_hull.h
../include/mc_rbdyn/surface_utils.h
../include/mc_rbdyn/Collision.h
../include/mc_rbdyn/ForceSensor.h
../include/mc_rbdyn/Flexibility.h
../include/mc_rbdyn/Springs.h
../include/mc_rbdyn/Base.h
../include/mc_rbdyn/BodySensor.h
../include/mc_rbdyn/configuration_io.h
)
set(mc_rbdyn_HDR ${mc_rbdyn_HDR} PARENT_SCOPE)

add_library(mc_rbdyn SHARED ${mc_rbdyn_SRC} ${mc_rbdyn_HDR})
set_target_properties(mc_rbdyn PROPERTIES COMPILE_FLAGS "-DMC_RBDYN_EXPORTS")
pkg_config_use_dependency(mc_rbdyn mc_rbdyn_urdf)
pkg_config_use_dependency(mc_rbdyn Tasks)
target_link_libraries(mc_rbdyn ${Boost_LIBRARIES} ${GEOS_LIBRARY} mc_rtc_loader mc_rtc_utils)
# qhull
target_include_directories(mc_rbdyn SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../3rd-party/qhull/src")
target_link_libraries(mc_rbdyn qhullcpp)
target_link_libraries(mc_rbdyn qhullstatic_r)
if(NOT WIN32)
  pkg_config_append_libs(${Boost_LIBRARIES})
endif()
pkg_config_append_libs(mc_rbdyn)
pkg_config_append_libs(${GEOS_LIBRARY})

add_subdirectory(mc_robots)

set(mc_solver_SRC
mc_solver/BoundedSpeedConstr.cpp
mc_solver/CollisionsConstraint.cpp
mc_solver/CoMIncPlaneConstr.cpp
mc_solver/ContactConstraint.cpp
mc_solver/DynamicsConstraint.cpp
mc_solver/KinematicsConstraint.cpp
mc_solver/QPSolver.cpp
)
set(mc_solver_SRC ${mc_solver_SRC} PARENT_SCOPE)

set(mc_solver_HDR
../include/mc_solver/api.h
../include/mc_solver/msg/Contact.h
../include/mc_solver/msg/QPResult.h
../include/mc_solver/msg/Robot.h
../include/mc_solver/BoundedSpeedConstr.h
../include/mc_solver/CollisionsConstraint.h
../include/mc_solver/CoMIncPlaneConstr.h
../include/mc_solver/ConstraintSet.h
../include/mc_solver/ContactConstraint.h
../include/mc_solver/DynamicsConstraint.h
../include/mc_solver/KinematicsConstraint.h
../include/mc_solver/QPSolver.h
../include/mc_solver/GenericLoader.h
../include/mc_solver/GenericLoader.hpp
)
set(mc_solver_HDR ${mc_solver_HDR} PARENT_SCOPE)

add_library(mc_solver SHARED ${mc_solver_SRC} ${mc_solver_HDR})
set_target_properties(mc_solver PROPERTIES COMPILE_FLAGS "-DMC_SOLVER_EXPORTS")
pkg_config_use_dependency(mc_solver Tasks)
pkg_config_append_libs(mc_solver)
target_link_libraries(mc_solver mc_rbdyn)

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
pkg_config_append_libs(mc_trajectory)

set(mc_tasks_SRC
mc_tasks/PositionTask.cpp
mc_tasks/SurfaceTransformTask.cpp
mc_tasks/OrientationTask.cpp
mc_tasks/VectorOrientationTask.cpp
mc_tasks/EndEffectorTask.cpp
mc_tasks/RelativeEndEffectorTask.cpp
mc_tasks/CoMTask.cpp
mc_tasks/MetaTask.cpp
mc_tasks/StabilityTask.cpp
mc_tasks/AddRemoveContactTask.cpp
mc_tasks/MoveContactTask.cpp
mc_tasks/TrajectoryTask.cpp
mc_tasks/ComplianceTask.cpp
mc_tasks/AdmittanceTask.cpp
mc_tasks/CoPTask.cpp
mc_tasks/GazeTask.cpp
mc_tasks/PositionBasedVisServoTask.cpp
mc_tasks/PostureTask.cpp
)
set(mc_tasks_SRC ${mc_tasks_SRC} PARENT_SCOPE)

set(mc_tasks_HDR
../include/mc_tasks/AddRemoveContactTask.h
../include/mc_tasks/api.h
../include/mc_tasks/CoMTask.h
../include/mc_tasks/EndEffectorTask.h
../include/mc_tasks/MetaTask.h
../include/mc_tasks/MetaTaskLoader.h
../include/mc_tasks/MoveContactTask.h
../include/mc_tasks/OrientationTask.h
../include/mc_tasks/VectorOrientationTask.h
../include/mc_tasks/PositionTask.h
../include/mc_tasks/SurfaceTransformTask.h
../include/mc_tasks/RelativeEndEffectorTask.h
../include/mc_tasks/SmoothTask.h
../include/mc_tasks/StabilityTask.h
../include/mc_tasks/TrajectoryTask.h
../include/mc_tasks/TrajectoryTaskGeneric.h
../include/mc_tasks/TrajectoryTaskGeneric.hpp
../include/mc_tasks/ComplianceTask.h
../include/mc_tasks/AdmittanceTask.h
../include/mc_tasks/CoPTask.h
../include/mc_tasks/GazeTask.h
../include/mc_tasks/PositionBasedVisServoTask.h
../include/mc_tasks/PostureTask.h
)
set(mc_tasks_HDR ${mc_tasks_HDR} PARENT_SCOPE)

add_library(mc_tasks SHARED ${mc_tasks_SRC} ${mc_tasks_HDR})
set_target_properties(mc_tasks PROPERTIES COMPILE_FLAGS "-DMC_TASKS_EXPORTS")
target_link_libraries(mc_tasks mc_solver mc_rbdyn mc_trajectory mc_rtc_utils)
target_link_libraries(mc_tasks ${Boost_LIBRARIES})
pkg_config_use_dependency(mc_tasks mc_rbdyn_urdf)
pkg_config_use_dependency(mc_tasks Tasks)
pkg_config_append_libs(mc_tasks)

add_library(mc_rtc_ros SHARED mc_rtc/ros.cpp ../include/mc_rtc/ros.h ../include/mc_rtc/ros_api.h)
set_target_properties(mc_rtc_ros PROPERTIES COMPILE_FLAGS "-DMC_RTC_ROS_EXPORTS")
target_link_libraries(mc_rtc_ros mc_rbdyn)
if(${ROSCPP_FOUND})
  pkg_config_use_dependency(mc_rtc_ros roscpp)
  pkg_config_use_dependency(mc_rtc_ros nav_msgs)
  pkg_config_use_dependency(mc_rtc_ros sensor_msgs)
  pkg_config_use_dependency(mc_rtc_ros tf2_ros)
  pkg_config_use_dependency(mc_rtc_ros mc_rtc_msgs)
endif()
pkg_config_append_libs(mc_rtc_ros)

set(mc_control_SRC
mc_control/ForceContactSensor.cpp
mc_control/SimulationContactSensor.cpp
mc_control/generic_gripper.cpp
mc_control/mc_controller.cpp
mc_control/mc_python_controller.cpp
mc_control/mc_global_controller.cpp
mc_control/mc_global_controller_configuration.cpp
mc_control/mc_global_controller_services.cpp
mc_control/mc_completion_criteria.cpp
)
set(mc_control_SRC ${mc_control_SRC} PARENT_SCOPE)

set(mc_control_HDR
../include/mc_control/api.h
../include/mc_control/Configuration.h
../include/mc_control/ContactSensor.h
../include/mc_control/ForceContactSensor.h
../include/mc_control/generic_gripper.h
../include/mc_control/mc_controller.h
../include/mc_control/mc_python_controller.h
../include/mc_control/SimulationContactSensor.h
../include/mc_control/mc_global_controller.h
../include/mc_control/mc_completion_criteria.h
)
set(mc_control_HDR ${mc_control_HDR} PARENT_SCOPE)

add_library(mc_control SHARED ${mc_control_SRC} ${mc_control_HDR})
set_target_properties(mc_control PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
target_link_libraries(mc_control mc_tasks mc_solver mc_rtc_ros mc_rtc_loader mc_rtc_utils)
if(${HRPSYS_BASE_FOUND})
  target_link_libraries(mc_control hrpIo)
endif()
target_link_libraries(mc_control ${Boost_LIBRARIES})
pkg_config_use_dependency(mc_control mc_rbdyn_urdf)
pkg_config_use_dependency(mc_control Tasks)
pkg_config_append_libs(mc_control)

if(FLATC)
  set_source_files_properties(../include/mc_rtc/log/serialization/MCLog_generated.h PROPERTIES GENERATED 1)
  add_dependencies(mc_rtc_utils generate_mc_log_code)
  add_dependencies(mc_solver generate_mc_log_code)
  add_dependencies(mc_tasks generate_mc_log_code)
  add_dependencies(mc_control generate_mc_log_code)
endif()

add_subdirectory(mc_control)

set(MC_LIBRARIES mc_rbdyn mc_solver mc_tasks mc_control mc_trajectory mc_rtc_ros mc_rtc_loader mc_rtc_utils)
foreach(MC_LIB ${MC_LIBRARIES})
  if(GENERATE_COVERAGE)
    target_compile_options(${MC_LIB} PRIVATE "--coverage")
    set_target_properties(${MC_LIB} PROPERTIES LINK_FLAGS "--coverage")
  endif()
  set_target_properties(${MC_LIB} PROPERTIES SOVERSION 0 VERSION 0.1.0)
endforeach()

install(TARGETS ${MC_LIBRARIES}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

