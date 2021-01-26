export ROS_DISTRO=noetic
export APT_DEPENDENCIES="curl wget cmake build-essential gfortran doxygen cython cython3 python-nose python3-nose python-numpy python3-numpy python-coverage python3-coverage python-setuptools python3-setuptools libeigen3-dev doxygen doxygen-latex libboost-all-dev libtinyxml2-dev libgeos++-dev libnanomsg-dev libyaml-cpp-dev libltdl-dev qt5-default libqwt-qt5-dev python3-matplotlib python3-pyqt5"
if $BUILD_BENCHMARKS
then
  export APT_DEPENDENCIES="$APT_DEPENDENCIES libbenchmark-dev"
fi

mc_rtc_extra_steps()
{
  curl https://bootstrap.pypa.io/2.7/get-pip.py -o get-pip.py && sudo python get-pip.py && rm -f get-pip.py
  sudo pip install matplotlib
  export MC_LOG_UI_PYTHON_EXECUTABLE=python3
}
