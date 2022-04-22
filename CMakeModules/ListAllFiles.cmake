# List all files managed by the project
set(MC_RTC_ALL_FILES)
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
file(STRINGS "${PROJECT_SOURCE_DIR}/.gitignore" IGNORE_FILTERS_IN)
set(IGNORE_FILTERS)
foreach(IGNORE_FILTER ${IGNORE_FILTERS_IN})
  string(REPLACE "*" ".*" IGNORE_FILTER "${IGNORE_FILTER}")
  set(IGNORE_FILTER "^${IGNORE_FILTER}$")
  list(APPEND IGNORE_FILTERS "${IGNORE_FILTER}")
endforeach()

macro(HANDLE_FILES VAR)
  foreach(F ${${VAR}})
    if(NOT IS_DIRECTORY ${F})
      set(IS_VALID TRUE)
      foreach(IGNORE_FILTER ${IGNORE_FILTERS})
        string(REGEX MATCH "${IGNORE_FILTER}" OUT "${F}")
        if(OUT)
          set(IS_VALID FALSE)
        endif()
      endforeach()
      if(${IS_VALID} AND EXISTS "${F}")
        list(APPEND MC_RTC_ALL_FILES ${F})
      endif()
    endif()
  endforeach()
endmacro()

file(GLOB ROOT_ALL_FILES "${PROJECT_SOURCE_DIR}/*")
HANDLE_FILES(ROOT_ALL_FILES)
foreach(SUBDIR ${MC_RTC_SUBDIRS})
  file(GLOB_RECURSE SUBDIR_ALL_FILES "${PROJECT_SOURCE_DIR}/${SUBDIR}/*")
  HANDLE_FILES(SUBDIR_ALL_FILES)
endforeach()
