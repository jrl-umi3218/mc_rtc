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
mc_rbdyn/robot.cpp
mc_rbdyn/stance.cpp
mc_rbdyn/StanceConfig.cpp
mc_rbdyn/PolygonInterpolator.cpp
mc_rbdyn/polygon_utils.cpp
)

add_library(mc_rbdyn SHARED ${mc_rbdyn_SRC})
pkg_config_use_dependency(mc_rbdyn mc_rbdyn_urdf_cpp)
pkg_config_use_dependency(mc_rbdyn Tasks)
pkg_config_use_dependency(mc_rbdyn jsoncpp)
target_link_libraries(mc_rbdyn ${Boost_LIBRARIES} ${GEOS_LIBRARY})
pkg_config_append_libs(${Boost_LIBRARIES})
pkg_config_append_libs(mc_rbdyn)
pkg_config_append_libs(${GEOS_LIBRARY})

set(mc_robots_SRC
mc_robots/hrp2_drc.cpp
mc_robots/env.cpp
mc_robots/polaris_ranger.cpp
mc_robots/polaris_ranger_egress.cpp
)

add_library(mc_robots SHARED ${mc_robots_SRC})
target_link_libraries(mc_robots mc_rbdyn)
pkg_config_append_libs(mc_robots)

set(mc_solver_SRC
mc_solver/contact_util.cpp
mc_solver/qpsolver.cpp
)

add_library(mc_solver SHARED ${mc_solver_SRC})
pkg_config_use_dependency(mc_solver Tasks)
pkg_config_append_libs(mc_solver)

set(mc_trajectory_SRC
mc_trajectory/BSplineTrajectory.cpp
mc_trajectory/spline_utils.cpp
)

add_library(mc_trajectory SHARED ${mc_trajectory_SRC})
pkg_config_append_libs(mc_trajectory)

set(mc_tasks_SRC
mc_tasks/OrientationTask.cpp
mc_tasks/EndEffectorTask.cpp
mc_tasks/RelativeEndEffectorTask.cpp
mc_tasks/CoMTask.cpp
mc_tasks/MetaTask.cpp
mc_tasks/StabilityTask.cpp
mc_tasks/AddRemoveContactTask.cpp
mc_tasks/MoveContactTask.cpp
mc_tasks/TrajectoryTask.cpp
)

add_library(mc_tasks SHARED ${mc_tasks_SRC})
target_link_libraries(mc_tasks mc_solver mc_rbdyn mc_trajectory)
pkg_config_append_libs(mc_tasks)

add_library(mc_rtc_ros SHARED mc_rtc/ros.cpp)
if(${ROSCPP_FOUND})
  pkg_config_use_dependency(mc_rtc_ros roscpp)
  pkg_config_use_dependency(mc_rtc_ros sensor_msgs)
  pkg_config_use_dependency(mc_rtc_ros tf2_ros)
endif()
pkg_config_append_libs(mc_rtc_ros)

set(mc_drc_controller_SRC
mc_control/ForceContactSensor.cpp
mc_control/SimulationContactSensor.cpp
mc_control/generic_gripper.cpp
mc_control/mc_virtual_controller.cpp
mc_control/mc_controller.cpp
mc_control/mc_posture_controller.cpp
mc_control/mc_body6d_controller.cpp
mc_control/mc_bci_self_interact_controller.cpp
mc_control/mc_com_controller.cpp
mc_control/MCSeqPublisher.cpp
mc_control/mc_seq_controller.cpp
mc_control/mc_seq_steps.cpp
mc_control/mc_mrqp_controller.cpp
mc_control/mc_driving_controller.cpp
mc_control/mc_egress_controller.cpp
mc_control/mc_egress_mrqp_controller.cpp
mc_control/mc_global_controller.cpp
mc_control/mc_global_controller_services.cpp
)
add_library(mc_drc_controller SHARED ${mc_drc_controller_SRC})
target_link_libraries(mc_drc_controller mc_tasks mc_solver mc_robots hrpIo mc_rtc_ros)
pkg_config_append_libs(mc_drc_controller)

set(MCControl_SRC
mc_rtc/MCControlServiceSVC_impl.cpp
mc_rtc/MCControl.cpp
${CMAKE_BINARY_DIR}/idl/MCControlServiceSK.cc
)

set_source_files_properties(${CMAKE_BINARY_DIR}/idl/MCControlServiceSK.cc
                            PROPERTIES
                              GENERATED 1
                              COMPILE_FLAGS -Wno-error
                           )

include_directories(${CMAKE_BINARY_DIR}/idl)

add_library(MCControl SHARED ${MCControl_SRC})
add_dependencies(MCControl generate_mc_idl_cpp)
set_target_properties(MCControl PROPERTIES PREFIX "")
pkg_config_use_dependency(MCControl hrpsys-base)
target_link_libraries(MCControl mc_rtc_ros mc_drc_controller)

add_executable(MCControlComp mc_rtc/MCControlComp.cpp)
target_link_libraries(MCControlComp MCControl)

install(TARGETS MCControl MCControlComp mc_rbdyn mc_solver mc_robots mc_tasks mc_drc_controller mc_trajectory mc_rtc_ros
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

