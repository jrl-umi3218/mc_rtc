#FIXME very crude initial cmake

cmake_minimum_required(VERSION 2.8)

include(idl.cmake)

project(mc_rtc_core)

add_definitions(-std=c++0x)
add_definitions(-mfpmath=sse -msse2)
#add_definitions(-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
#add_definitions(-Wno-sign-conversion -Wno-shorten-64-to-32)
#add_definitions(-Werror)

find_package(PkgConfig)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
include_directories(${OPENRTM_INCLUDE_DIRS})
add_definitions(${OPENRTM_CFLAGS})
link_directories(${OPENRTM_LIBRARY_DIRS})

pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

pkg_check_modules(HRPSYSBASE REQUIRED hrpsys-base)
include_directories(${HRPSYSBASE_INCLUDE_DIRS})
link_directories(${HRPSYSBASE_LIBRARY_DIRS})

include_directories(include)

OMNIIDL_INCLUDE_DIRECTORIES("idl")
add_subdirectory(idl)

set(mc_rbdyn_urdf_SRC
./src/mc_rbdyn_urdf/urdf.cpp
)

add_library(mc_rbdyn_urdf SHARED ${mc_rbdyn_urdf_SRC})
target_link_libraries(mc_rbdyn_urdf RBDyn tinyxml2)

set(mc_rbdyn_SRC
./src/mc_rbdyn/SCHAddon.cpp
./src/mc_rbdyn/contact_transform.cpp
./src/mc_rbdyn/Surface.cpp
./src/mc_rbdyn/PlanarSurface.cpp
./src/mc_rbdyn/CylindricalSurface.cpp
./src/mc_rbdyn/GripperSurface.cpp
./src/mc_rbdyn/surface_utils.cpp
./src/mc_rbdyn/surface_hull.cpp
./src/mc_rbdyn/Contact.cpp
./src/mc_rbdyn/robot.cpp
./src/mc_rbdyn/stance.cpp
./src/mc_rbdyn/StanceConfig.cpp
)

add_library(mc_rbdyn SHARED ${mc_rbdyn_SRC})
target_link_libraries(mc_rbdyn RBDyn tinyxml2 boost_filesystem boost_system geos jsoncpp sch-core Tasks)

set(mc_robots_SRC
./src/mc_robots/hrp2_drc.cpp
./src/mc_robots/env.cpp
./src/mc_robots/polaris_ranger.cpp
./src/mc_robots/polaris_ranger_egress.cpp
)
add_library(mc_robots SHARED ${mc_robots_SRC})
target_link_libraries(mc_robots mc_rbdyn mc_rbdyn_urdf)

set(mc_solver_SRC
./src/mc_solver/contact_util.cpp
./src/mc_solver/qpsolver.cpp
)
add_library(mc_solver SHARED ${mc_solver_SRC})
target_link_libraries(mc_solver RBDyn Tasks)

set(mc_trajectory_SRC
./src/mc_trajectory/BSplineTrajectory.cpp
./src/mc_trajectory/spline_utils.cpp
)
add_library(mc_trajectory SHARED ${mc_trajectory_SRC})

set(mc_tasks_SRC
./src/mc_tasks/OrientationTask.cpp
./src/mc_tasks/EndEffectorTask.cpp
./src/mc_tasks/CoMTask.cpp
./src/mc_tasks/MetaTask.cpp
./src/mc_tasks/StabilityTask.cpp
./src/mc_tasks/AddRemoveContactTask.cpp
./src/mc_tasks/MoveContactTask.cpp
./src/mc_tasks/TrajectoryTask.cpp
)
add_library(mc_tasks SHARED ${mc_tasks_SRC})
target_link_libraries(mc_tasks mc_solver mc_rbdyn mc_trajectory)

set(mc_drc_controller_SRC
./src/mc_control/ForceContactSensor.cpp
./src/mc_control/SimulationContactSensor.cpp
./src/mc_control/generic_gripper.cpp
./src/mc_control/mc_virtual_controller.cpp
./src/mc_control/mc_controller.cpp
./src/mc_control/mc_posture_controller.cpp
./src/mc_control/mc_body6d_controller.cpp
./src/mc_control/mc_com_controller.cpp
./src/mc_control/mc_seq_controller.cpp
./src/mc_control/mc_seq_steps.cpp
./src/mc_control/mc_mrqp_controller.cpp
./src/mc_control/mc_driving_controller.cpp
./src/mc_control/mc_egress_controller.cpp
./src/mc_control/mc_egress_mrqp_controller.cpp
./src/mc_control/mc_drc_controller.cpp
./src/mc_control/mc_drc_controller_services.cpp
)
add_library(mc_drc_controller SHARED ${mc_drc_controller_SRC})
target_link_libraries(mc_drc_controller mc_tasks mc_solver mc_robots hrpIo)

set(MCControl_SRC
./src/MCControlServiceSVC_impl.cpp
./src/MCControl.cpp
${CMAKE_BINARY_DIR}/idl/MCControlServiceSK.cc
)

set_source_files_properties(${CMAKE_BINARY_DIR}/idl/MCControlServiceSK.cc
                            PROPERTIES GENERATED 1)

include_directories(${CMAKE_BINARY_DIR}/idl)

add_library(MCControl SHARED ${MCControl_SRC})
add_dependencies(MCControl generate_mc_idl_cpp)
set_target_properties(MCControl PROPERTIES PREFIX "")
target_link_libraries(MCControl ${OPENRTM_LIBRARIES} ${Boost_LIBRARIES} mc_rbdyn mc_rbdyn_urdf mc_drc_controller)

add_executable(MCControlComp ./src/MCControlComp.cpp)
target_link_libraries(MCControlComp MCControl)

add_executable(testLoadJSON ./src/utils/testLoadJSON.cpp)
target_link_libraries(testLoadJSON mc_rbdyn mc_rbdyn_urdf)

add_executable(testBSplineTrajectory ./tests/testBSplineTrajectory.cpp)
target_link_libraries(testBSplineTrajectory mc_trajectory)

install(TARGETS MCControl MCControlComp mc_rbdyn mc_rbdyn_urdf mc_solver mc_robots mc_tasks mc_drc_controller mc_trajectory
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

set_source_files_properties(${CMAKE_BINARY_DIR}/idl/MCControlService_idl.py
                            PROPERTIES GENERATED 1)

install(FILES ${CMAKE_BINARY_DIR}/idl/MCControlService_idl.py DESTINATION lib/python2.7/dist-packages/hrpsys)
#install(FILES python/mc_rtc_interface.py DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
