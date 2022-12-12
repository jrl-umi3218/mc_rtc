export CMAKE_ADDITIONAL_OPTIONS="-DCMAKE_MACOSX_RPATH:BOOL=ON"
export PYTHON_FORCE_PYTHON2="false"
export PYTHON_FORCE_PYTHON3="true"
export PYTHON_BUILD_PYTHON2_AND_PYTHON3="false"
export MC_LOG_UI_PYTHON_EXECUTABLE=python3
export SYSTEM_HAS_SPDLOG=ON
export BREW_DEPENDENCIES="coreutils pkg-config gnu-sed wget python cmake doxygen libtool tinyxml2 geos boost eigen nanomsg yaml-cpp qt qwt pyqt gcc spdlog ninja libnotify"
if $BUILD_BENCHMARKS
then
  export BREW_DEPENDENCIES="$BREW_DEPENDENCIES google-benchmark"
fi

export PIP_DEPENDENCIES="Cython coverage pytest numpy matplotlib"
if $WITH_ROS_SUPPORT && [ -z $ROS_DISTRO ]
then
  echo "ROS support is disabled as ROS was not detected. If you have ROS, please source the setup script before running this script."
  export WITH_ROS_SUPPORT="false"
fi


mc_rtc_extra_steps()
{
  # Temparary fix for the macOS setup on github actions
  brew unlink gfortran && brew link gfortran
}
