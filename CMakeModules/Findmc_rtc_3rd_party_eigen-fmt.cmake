if(TARGET eigen-fmt::eigen-fmt)
  return()
endif()

# Try to find system eigen-fmt first To avoid having to deal with PID dependencies, we
# manually make the header-only target eigen-fmt::eigen-fmt available and avoid using
# the project's CMakeLists.txt
find_package(eigen-fmt CONFIG QUIET)

if(NOT TARGET eigen-fmt::eigen-fmt)
  include(FetchContent)
  FetchContent_Declare(
    eigen-fmt_proj
    QUIET
    GIT_REPOSITORY https://gite.lirmm.fr/rpc/utils/eigen-fmt
    # upcoming v1.0.5 with fmt range formatter disabled for eigen types
    GIT_TAG 74010fe68f0337e6bf0726dc958511414bef531f
    # point to a directory that does not exist to avoid building the existing project's
    # CMakeLists.txt See
    # https://discourse.cmake.org/t/prevent-fetchcontent-makeavailable-to-execute-cmakelists-txt/12704/12
    SOURCE_SUBDIR skip-cmake-build
  )
  FetchContent_MakeAvailable(eigen-fmt_proj) # Only fetch, don't build

  # Define interface library for header-only eigen-fmt
  if(NOT TARGET eigen-fmt::eigen-fmt)
    add_library(eigen-fmt INTERFACE IMPORTED)

    target_include_directories(
      eigen-fmt INTERFACE "${eigen-fmt_proj_SOURCE_DIR}/include/eigen-fmt"
    )

    add_library(eigen-fmt::eigen-fmt ALIAS eigen-fmt)
  endif()
endif()
