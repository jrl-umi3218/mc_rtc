#################
# -- libltdl -- #
#################

# On WIN32 platform, compile and install our own libltdl
if(WIN32)
  add_library(ltdl SHARED ltdl/ltdl.c)
  # Generally import ltdl symbols
  add_definitions(-DLIBLTDL_DLL_IMPORT)
  pkg_config_append_cflags(-DLIBLTDL_DLL_IMPORT)
  set_target_properties(ltdl PROPERTIES COMPILE_FLAGS "-DDLL_EXPORT")
# On other platforms check that the header is available in the system
else()
  include(CheckIncludeFileCXX)
  check_include_file_cxx(ltdl.h LTDL_H_FOUND)
  if(NOT LTDL_H_FOUND)
    MESSAGE(FATAL_ERROR "Failed to find ltdl.h, check that libtool ltdl is installed")
  endif()
endif()

pkg_config_append_libs(ltdl)
