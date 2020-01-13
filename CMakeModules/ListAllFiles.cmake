# List all files managed by the project
set(MC_RTC_ALL_FILES)
file(GLOB ROOT_ALL_FILES "${PROJECT_SOURCE_DIR}/*")
foreach(F ${ROOT_ALL_FILES})
  if(NOT IS_DIRECTORY ${F})
    list(APPEND MC_RTC_ALL_FILES ${F})
  endif()
endforeach()
set(MC_RTC_SUBDIRS
  3rd-party
  benchmarks
  binding
  cmake
  CMakeModules
  debian
  doc
  etc
  include
  plugins
  src
  tests
  utils
)
foreach(SUBDIR ${MC_RTC_SUBDIRS})
  file(GLOB_RECURSE SUBDIR_ALL_FILES "${PROJECT_SOURCE_DIR}/${SUBDIR}/*")
  foreach(F ${SUBDIR_ALL_FILES})
    if(NOT IS_DIRECTORY ${F})
      list(APPEND MC_RTC_ALL_FILES ${F})
    endif()
  endforeach()
endforeach()
