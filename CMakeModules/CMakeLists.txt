#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(MODULE_FILES
  Findmc_rtc_3rd_party_geos.cmake
  Findmc_rtc_3rd_party_hpp-spline.cmake
  Findmc_rtc_3rd_party_nanomsg.cmake
)

if(NOT WIN32)
  list(APPEND MODULE_FILES Findmc_rtc_3rd_party_ltdl.cmake)
endif()

install(FILES ${MODULE_FILES} DESTINATION "${CONFIG_INSTALL_DIR}")
foreach(F ${MODULE_FILES})
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}
include(\"\${CMAKE_CURRENT_LIST_DIR}/${F}\")")
endforeach()
set(PACKAGE_EXTRA_MACROS ${PACKAGE_EXTRA_MACROS} PARENT_SCOPE)
