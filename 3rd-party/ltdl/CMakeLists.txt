#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# On WIN32 platform, compile and install our own libltdl
if(WIN32)
  set(LTDL ltdl PARENT_SCOPE)
  set(LTDL ltdl)
  add_library(${LTDL} SHARED ltdl.c ltdl.h)
  set_target_properties(${LTDL} PROPERTIES FOLDER 3rd-party)
  # Generally import ltdl symbols
  target_compile_definitions(${LTDL} PRIVATE -DDLL_EXPORT -DHAVE_STDLIB_H=1 -DHAVE_ERRNO_H=1 -DHAVE_MEMORY_H=1 -DHAVE_CTYPE_H=1 -DHAVE_STRING_H=1 -DHAVE_STDIO_H=1 -DHAVE_ASSERT_H=1 -D__WINDOWS__ -D__WIN32__)
  target_include_directories(${LTDL} INTERFACE $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
  install(FILES ltdl.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
  install(TARGETS ${LTDL}
          EXPORT "${TARGETS_EXPORT_NAME}"
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
# On other platforms check that the header is available in the system
else()
  set(LTDL mc_rtc_3rd_party::ltdl PARENT_SCOPE)
  find_package(mc_rtc_3rd_party_ltdl REQUIRED)
endif()
