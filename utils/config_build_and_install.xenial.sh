export ROS_DISTRO=kinetic
export APT_DEPENDENCIES="wget cmake build-essential gfortran doxygen cython cython3 python-pip python-nose python3-nose python-numpy python3-numpy python-coverage python3-coverage python-setuptools python3-setuptools libeigen3-dev doxygen doxygen-latex libboost-all-dev libtinyxml2-dev libgeos++-dev libnanomsg-dev libyaml-cpp-dev libltdl-dev python-git python-pyqt5 qt5-default libqwt-qt5-dev python-matplotlib"

mc_rtc_extra_steps()
{
  if ${BUILD_BENCHMARKS}
  then
    # install google benchmark library
    git clone https://github.com/google/benchmark.git $SOURCE_DIR/benchmark
    cd $SOURCE_DIR/benchmark
    mkdir build && cd build
    cmake  -DBENCHMARK_ENABLE_GTEST_TESTS:BOOL=OFF ..
    make -j${BUILD_CORE}
    sudo make install
  fi
}
