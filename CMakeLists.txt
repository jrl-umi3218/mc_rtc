#FIXME very crude initial cmake

cmake_minimum_required(VERSION 2.8)

project(mc_rtc_core)

include_directories(/usr/include/openrtm-1.1 /usr/include/openrtm-1.1/rtm/idl /usr/include/coil-1.1)

set(MCControl_SRC
./src/MCControlServiceSK.cc
./src/MCControlServiceSVC_impl.cpp
./src/MCControl.cpp
)

add_library(MCControl SHARED ${MCControl_SRC})
set_target_properties(MCControl PROPERTIES PREFIX "")
target_link_libraries(MCControl RTC coil omniORB4 omnithread omniDynamic4)

add_executable(MCControlComp ./src/MCControlComp.cpp)
target_link_libraries(MCControlComp MCControl)

install(TARGETS MCControl MCControlComp
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES python/MCControlService_idl.py DESTINATION /usr/local/lib/python2.7/dist-packages/hrpsys)
