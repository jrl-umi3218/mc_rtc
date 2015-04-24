#FIXME very crude initial cmake

cmake_minimum_required(VERSION 2.8)

project(mc_rtc_core)

add_definitions(-std=c++11)
#add_definitions(-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
#add_definitions(-Wno-sign-conversion -Wno-shorten-64-to-32)
#add_definitions(-Werror)

include_directories(/usr/include/openrtm-1.1 /usr/include/openrtm-1.1/rtm/idl /usr/include/coil-1.1 /usr/include/eigen3)

include_directories(include)

set(mc_rbdyn_urdf_SRC
./src/mc_rbdyn_urdf/urdf.cpp
)

add_library(mc_rbdyn_urdf SHARED ${mc_rbdyn_urdf_SRC})
target_link_libraries(mc_rbdyn_urdf RBDyn tinyxml2)

set(mc_rbdyn_SRC
./src/mc_rbdyn/SCHAddon.cpp
./src/mc_rbdyn/contact_transform.cpp
./src/mc_rbdyn/surface.cpp
./src/mc_rbdyn/surface_hull.cpp
./src/mc_rbdyn/contact.cpp
./src/mc_rbdyn/robot.cpp
./src/mc_rbdyn/stance.cpp
)

add_library(mc_rbdyn SHARED ${mc_rbdyn_SRC})
target_link_libraries(mc_rbdyn RBDyn tinyxml2 boost_filesystem boost_system geos jsoncpp sch-core Tasks)

set(mc_robots_SRC
./src/mc_robots/hrp2_drc.cpp
./src/mc_robots/env.cpp
)
add_library(mc_robots SHARED ${mc_robots_SRC})
target_link_libraries(mc_robots mc_rbdyn mc_rbdyn_urdf)

set(mc_control_solver_SRC
./src/mc_control/mc_solver/contact_util.cpp
./src/mc_control/mc_solver/qpsolver.cpp
)
add_library(mc_control_solver SHARED ${mc_control_solver_SRC})
target_link_libraries(mc_control_solver RBDyn Tasks)

set(mc_drc_controller_SRC
./src/mc_control/mc_drc_controller.cpp
./src/mc_control/mc_drc_controller_services.cpp
)
add_library(mc_drc_controller SHARED ${mc_drc_controller_SRC})
target_link_libraries(mc_drc_controller mc_control_solver mc_robots)

set(MCControl_SRC
./src/MCControlServiceSVC_impl.cpp
./src/MCControl.cpp
)
add_library(MCControl SHARED ${MCControl_SRC})
set_target_properties(MCControl PROPERTIES PREFIX "")
target_link_libraries(MCControl RTC coil omniORB4 omnithread omniDynamic4 mc_rbdyn mc_rbdyn_urdf mc_drc_controller)

add_executable(MCControlComp ./src/MCControlComp.cpp)
target_link_libraries(MCControlComp MCControl)

install(TARGETS MCControl MCControlComp mc_rbdyn mc_rbdyn_urdf mc_control_solver mc_robots mc_drc_controller
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES python/MCControlService_idl.py DESTINATION lib/python2.7/dist-packages/hrpsys)
install(FILES python/mc_rtc_interface.py DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
