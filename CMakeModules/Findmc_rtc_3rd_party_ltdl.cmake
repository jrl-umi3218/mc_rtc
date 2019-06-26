#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find the ltdl library
#
# This can use LTDL_PREFIX as an hint
#
# Defines the mc_rtc_3rd_party::ltdl target

set(LTDL mc_rtc_3rd_party::ltdl)
if(NOT TARGET ${LTDL})

  if(NOT DEFINED LTDL_PREFIX)
    set(LTDL_PREFIX ${CMAKE_INSTALL_PREFIX})
  endif()
  
  find_path(LTDL_INCLUDE_DIR
    NAMES ltdl.h
    HINTS ${LTDL_PREFIX}
    )
  find_library(LTDL_LIBRARY
    NAMES ltdl
    PATHS ${LTDL_PREFIX}
    )
  
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(LTDL DEFAULT_MSG LTDL_LIBRARY LTDL_INCLUDE_DIR)
  mark_as_advanced(LTDL_INCLUDE_DIR LTDL_LIBRARY)
  if(LTDL_FOUND)
    add_library(${LTDL} INTERFACE IMPORTED GLOBAL)
    set_target_properties(${LTDL} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${LTDL_INCLUDE_DIR}
      INTERFACE_LINK_LIBRARIES ${LTDL_LIBRARY}
      )
  endif()

endif()
