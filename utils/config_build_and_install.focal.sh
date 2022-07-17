export ROS_DISTRO=noetic
export SYSTEM_HAS_SPDLOG=ON
export APT_DEPENDENCIES="curl wget cmake build-essential gfortran doxygen cython cython3 python3-pip python-nose python3-nose python-numpy python3-numpy python-coverage python3-coverage python-setuptools python3-setuptools libeigen3-dev doxygen doxygen-latex libboost-all-dev libtinyxml2-dev libgeos++-dev libnanomsg-dev libyaml-cpp-dev libltdl-dev qt5-default libqwt-qt5-dev python3-matplotlib python3-pyqt5 libspdlog-dev ninja-build"
if $BUILD_BENCHMARKS
then
  export APT_DEPENDENCIES="$APT_DEPENDENCIES libbenchmark-dev"
fi
export PYTHON_VERSION_MAJOR=`python -c 'import sys; print("{}".format(sys.version_info.major))'`
if [ "x$PYTHON_VERSION_MAJOR" == x2 ]
then
  export APT_DEPENDENCIES="$APT_DEPENDENCIES python-is-python2"
elif [ "x$PYTHON_VERSION_MAJOR" == x3 ]
then
  export APT_DEPENDENCIES="$APT_DEPENDENCIES python-is-python3"
fi

export MC_LOG_UI_PYTHON_EXECUTABLE=python3

mc_rtc_extra_steps()
{
  if [ "x$PYTHON_FORCE_PYTHON2" == xON || "x$PYTHON_BUILD_PYTHON2_AND_PYTHON3" == xON || "x$PYTHON_VERSION_MAJOR" == x2 ]
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && sudo python2 get-pip.py && rm -f get-pip.py
    sudo pip install matplotlib
    # pip3 gets overwritten by the get-pip.py script, but the python3-pip package remains installed. This usures that pip3 remains cleanly installed.
    sudo apt-get install --reinstall -y python3-pip
  fi
}
