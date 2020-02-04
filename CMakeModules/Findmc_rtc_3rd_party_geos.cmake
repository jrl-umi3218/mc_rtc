#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find the GEOS library
#
# In order:
# - try to find_package(geos) (upstream package)
# - try to use geos-config to find the geos installation prefix
# - try to find the geos library
#
# If the library is found, then you can use the GEOS::geos target
#
# This module will use GEOS_PREFIX as an hint to find either the geos-config
# executable or the geos library

if(NOT TARGET GEOS::geos)
find_package(geos QUIET)
if(NOT ${geos_FOUND})
  if(NOT DEFINED GEOS_PREFIX)
    set(GEOS_PREFIX ${CMAKE_INSTALL_PREFIX})
  endif()
  # Find the GEOS-config program
  find_program(GEOS_CONFIG geos-config
    /usr/local/bin
    /usr/bin
    ${GEOS_PREFIX}/bin
    ${CMAKE_INSTALL_PREFIX}/bin
  )
  if(GEOS_CONFIG)
    # Get GEOS_INSTALL_PREFIX from geos-config
    exec_program(${GEOS_CONFIG} ARGS --prefix OUTPUT_VARIABLE GEOS_INSTALL_PREFIX)
    find_library(GEOS_LIBRARY
      NAME geos
      HINTS ${GEOS_INSTALL_PREFIX}
    )
    if(NOT GEOS_LIBRARY)
      message(FATAL_ERROR "Found GEOS install prefix (${GEOS_INSTALL_PREFIX}) but no geos library")
    endif()
    message("-- Found GEOS by geos-config: ${GEOS_LIBRARY}")
  else()
    find_library(GEOS_LIBRARY
      NAMES geos
      HINTS ${GEOS_PREFIX} ${CMAKE_INSTALL_PREFIX}
    )
    if(GEOS_LIBRARY)
      get_filename_component(GEOS_INSTALL_PREFIX ${GEOS_LIBRARY} DIRECTORY)
      set(GEOS_INSTALL_PREFIX "${GEOS_INSTALL_PREFIX}/..")
      get_filename_component(GEOS_INSTALL_PREFIX ${GEOS_INSTALL_PREFIX} ABSOLUTE)
      message("-- Found GEOS library: ${GEOS_LIBRARY}")
    else()
      message(FATAL_ERROR "Could not find the geos package, geos-config or the geos library, either you are missing the dependency or you should provide the GEOS_PREFIX hint")
    endif()
  endif()
  add_library(GEOS::geos INTERFACE IMPORTED)
  set_target_properties(GEOS::geos PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${GEOS_INSTALL_PREFIX}/include
    INTERFACE_LINK_LIBRARIES ${GEOS_LIBRARY}
  )
endif()
endif()
