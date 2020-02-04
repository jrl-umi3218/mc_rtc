#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find the nanomsg library
#
# In order:
# - try to find_package(nanomsg) (upstream package)
# - try to find nanomsg or libnanomsg using pkg-config
#
# If the library is found, then you can use the nanomsg target
#

if(NOT TARGET nanomsg)
find_package(nanomsg QUIET)
if(NOT ${nanomsg_FOUND} OR NOT TARGET nanomsg)
  include(FindPkgConfig)
  pkg_search_module(NANOMSG nanomsg libnanomsg)
  if(NOT NANOMSG_FOUND)
    message(FATAL_ERROR "Could not find the nanomsg package using CMake package or pkg-config")
  endif()
  foreach(LIB ${NANOMSG_LIBRARIES})
    find_library(${LIB}_FULL_PATH NAME ${LIB} HINTS ${NANOMSG_LIBRARY_DIRS})
    list(APPEND NANOMSG_FULL_LIBRARIES ${${LIB}_FULL_PATH})
  endforeach()
  message("-- Found nanomsg libraries: ${NANOMSG_FULL_LIBRARIES}")
  add_library(nanomsg INTERFACE IMPORTED)
  set_target_properties(nanomsg PROPERTIES
    INTERFACE_LINK_LIBRARIES "${NANOMSG_FULL_LIBRARIES}"
  )
  if(NOT "${NANOMSG_INCLUDE_DIRS}" STREQUAL "")
    set_target_properties(nanomsg PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${NANOMSG_INCLUDE_DIRS}"
    )
    message("-- Found nanomsg include directories: ${NANOMSG_INCLUDE_DIRS}")
  endif()
endif()
endif()
