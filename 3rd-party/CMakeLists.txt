#################
# -- libltdl -- #
#################

# On WIN32 platform, compile and install our own libltdl
if(WIN32)
  add_library(ltdl SHARED ltdl/ltdl.c ltdl/ltdl.h)
  # Generally import ltdl symbols
  add_definitions(-DLIBLTDL_DLL_IMPORT)
  pkg_config_append_cflags(-DLIBLTDL_DLL_IMPORT)
  set_target_properties(ltdl PROPERTIES COMPILE_FLAGS "-DDLL_EXPORT -DHAVE_STDLIB_H=1 -DHAVE_ERRNO_H=1 -DHAVE_MEMORY_H=1 -DHAVE_CTYPE_H=1 -DHAVE_STRING_H=1 -DHAVE_STDIO_H=1 -DHAVE_ASSERT_H=1 -D__WINDOWS__ -D__WIN32__")
  install(FILES ltdl/ltdl.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
  install(TARGETS ltdl
                  RUNTIME DESTINATION bin
                  LIBRARY DESTINATION lib
                  ARCHIVE DESTINATION lib)
# On other platforms check that the header is available in the system
else()
  include(CheckIncludeFileCXX)
  check_include_file_cxx(ltdl.h LTDL_H_FOUND)
  if(NOT LTDL_H_FOUND)
    message(FATAL_ERROR "Failed to find ltdl.h, check that libtool ltdl is installed")
  endif()
endif()

pkg_config_append_libs(ltdl)

add_subdirectory(qhull)
