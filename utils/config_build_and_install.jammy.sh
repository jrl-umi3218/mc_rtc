# For now we disable ROS support on 22.04 maybe support both ROS2 humble/rolling later
export WITH_ROS_SUPPORT="false"
export ROS_DISTRO=
export SYSTEM_HAS_SPDLOG=ON
export APT_DEPENDENCIES="curl wget cmake build-essential gfortran doxygen cython3 python3-pip python3-nose python3-numpy python3-coverage python3-setuptools libeigen3-dev doxygen doxygen-latex libboost-all-dev libtinyxml2-dev libnanomsg-dev libyaml-cpp-dev libltdl-dev libqwt-qt5-dev python3-matplotlib python3-pyqt5 libspdlog-dev ninja-build git python-is-python3 graphviz libgeos++-dev"
if $BUILD_BENCHMARKS
then
  export APT_DEPENDENCIES="$APT_DEPENDENCIES libbenchmark-dev"
fi

# No more Python 2
export PYTHON_FORCE_PYTHON2="false"
export PYTHON_FORCE_PYTHON3="true"
export PYTHON_BUILD_PYTHON2_AND_PYTHON3="false"
export MC_LOG_UI_PYTHON_EXECUTABLE=python3

mc_rtc_extra_steps()
{
  # Fix GEOS installation: see https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=1010002
  if [ ! -f /usr/include/geos/geom/Coordinate.inl ]
  then
    if [ ! -d $SOURCE_DIR/geos ]
    then
      git clone --recursive https://github.com/libgeos/geos $SOURCE_DIR/geos
      cd $SOURCE_DIR/geos
      git checkout 3.10.2
    fi
    if $CLONE_ONLY
    then
      return
    fi
    ${REQUIRED_SUDO} cp $SOURCE_DIR/geos/include/geos/geom/*.inl /usr/include/geos/geom/
  fi
}
