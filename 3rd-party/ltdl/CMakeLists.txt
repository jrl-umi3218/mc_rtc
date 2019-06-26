#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# On WIN32 platform, compile and install our own libltdl
if(WIN32)
  set(LTDL mc_rtc_3rd_party::ltdl)
  add_library(${LTDL} SHARED ltdl/ltdl.c ltdl/ltdl.h)
  # Generally import ltdl symbols
  target_compile_definitions(${LTDL} PRIVATE -DDLL_EXPORT -DHAVE_STDLIB_H=1 -DHAVE_ERRNO_H=1 -DHAVE_MEMORY_H=1 -DHAVE_CTYPE_H=1 -DHAVE_STRING_H=1 -DHAVE_STDIO_H=1 -DHAVE_ASSERT_H=1 -D__WINDOWS__ -D__WIN32__)
  install(FILES ltdl/ltdl.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
  install(TARGETS ${LTDL}
          EXPORT "${TARGETS_EXPORT_NAME}"
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
# On other platforms check that the header is available in the system
else()
  find_package(mc_rtc_3rd_party_ltdl REQUIRED)
endif()
