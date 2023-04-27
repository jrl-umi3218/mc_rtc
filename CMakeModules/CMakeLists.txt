#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(PACKAGE_EXTRA_MACROS
    "${PACKAGE_EXTRA_MACROS}
find_package(PkgConfig)
list(APPEND CMAKE_MODULE_PATH \${CMAKE_CURRENT_LIST_DIR})"
)

macro(Add3rdPartyModule MODULE)
  install(FILES Findmc_rtc_3rd_party_${MODULE}.cmake
          DESTINATION "${CONFIG_INSTALL_DIR}"
  )
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}
find_package(mc_rtc_3rd_party_${MODULE} REQUIRED)"
  )
endmacro()

add3rdpartymodule(geos)

if(NOT MC_RTC_DISABLE_NETWORK)
  add3rdpartymodule(nanomsg)
endif()

if(NOT WIN32 AND NOT MC_RTC_BUILD_STATIC)
  add3rdpartymodule(ltdl)
endif()

if(NOT WIN32)
  add3rdpartymodule(libnotify)
endif()

set(PACKAGE_EXTRA_MACROS
    ${PACKAGE_EXTRA_MACROS}
    PARENT_SCOPE
)
