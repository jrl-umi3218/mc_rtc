export CASK_DEPENDENCIES="gfortran"
export BREW_DEPENDENCIES="coreutils pkg-config gnu-sed wget python cmake doxygen libtool tinyxml2 geos boost eigen nanomsg yaml-cpp qt qwt pyqt"
if $BUILD_BENCHMARKS
then
  export BREW_DEPENDENCIES="$BREW_DEPENDENCIES google-benchmark"
fi

export PIP_DEPENDENCIES="Cython coverage nose numpy matplotlib"
if $WITH_ROS_SUPPORT && [ -z $ROS_DISTRO ]
then
  echo "ROS support is disabled as ROS was not detected. If you have ROS, please source the setup script before running this script."
  export WITH_ROS_SUPPORT="false"
fi
