#FIXME very crude initial cmake

cmake_minimum_required(VERSION 2.8)

project(mc_rtc_core)

add_definitions(-std=c++11)

include_directories(/usr/include/openrtm-1.1 /usr/include/openrtm-1.1/rtm/idl /usr/include/coil-1.1 /usr/include/eigen3)

include_directories(include)

set(mc_rbdyn_urdf_SRC
./src/mc_rbdyn_urdf/urdf.cpp
)

add_library(mc_rbdyn_urdf SHARED ${mc_rbdyn_urdf_SRC})
target_link_libraries(mc_rbdyn_urdf RBDyn)


set(MCControl_SRC
./src/MCControlServiceSK.cc
./src/MCControlServiceSVC_impl.cpp
./src/MCControl.cpp
)

add_library(MCControl SHARED ${MCControl_SRC})
set_target_properties(MCControl PROPERTIES PREFIX "")
target_link_libraries(MCControl RTC coil omniORB4 omnithread omniDynamic4 mc_rbdyn_urdf)

add_executable(MCControlComp ./src/MCControlComp.cpp)
target_link_libraries(MCControlComp MCControl)

install(TARGETS MCControl MCControlComp mc_rbdyn_urdf
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES python/MCControlService_idl.py DESTINATION /usr/local/lib/python2.7/dist-packages/hrpsys)
