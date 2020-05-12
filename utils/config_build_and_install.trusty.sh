export ROS_DISTRO=indigo
export APT_DEPENDENCIES="wget cmake build-essential gfortran doxygen software-properties-common cython python-pip python-coverage python-numpy python-nose libtinyxml2-dev libboost-all-dev libgeos++-dev libgeos-dev libltdl-dev python-enum34 python-git qt4-default python-matplotlib"

mc_rtc_extra_steps()
{
  if [ ! -f /etc/apt/sources.list.d/pierre-gergondet_ppa-multi-contact-unstable-trusty.list ]
  then
    sudo add-apt-repository ppa:pierre-gergondet+ppa/multi-contact-unstable
    sudo apt-get update
    sudo apt-get upgrade cmake libeigen3-dev -qq
  fi
  if [ ! -d $SOURCE_DIR/yaml-cpp ]
  then
    git clone --recursive https://github.com/jbeder/yaml-cpp $SOURCE_DIR/yaml-cpp
  fi
  if [ ! -f $SOURCE_DIR/yaml-cpp/build/install_manifest.txt ] && $NOT_CLONE_ONLY
  then
    mkdir -p $SOURCE_DIR/yaml-cpp/build
    cd $SOURCE_DIR/yaml-cpp
    git checkout yaml-cpp-0.5.3
    cd build
    cmake ../ -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_SHARED_LIBS:BOOL=ON -DYAML_BUILD_SHARED_LIBS:BOOL=ON
    make -j${BUILD_CORE}
    sudo make install
  fi
  if [ ! -d $SOURCE_DIR/nanomsg ]
  then
    git clone --recursive https://github.com/nanomsg/nanomsg $SOURCE_DIR/nanomsg
  fi
  if [ ! -f $SOURCE_DIR/nanomsg/build/install_manifest.txt ] && $NOT_CLONE_ONLY
  then
    mkdir -p $SOURCE_DIR/nanomsg/build
    cd $SOURCE_DIR/nanomsg/build
    cmake ../ -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    make -j${BUILD_CORE}
    sudo make install
  fi
  if $WITH_ROS_SUPPORT
  then
    cd $SOURCE_DIR
    if [ ! -d $SOURCE_DIR/qwt-6.1.4 ]
    then
      wget -O qwt-6.1.4.tar.bz2 https://sourceforge.net/projects/qwt/files/qwt/6.1.4/qwt-6.1.4.tar.bz2/download
      tar xjf qwt-6.1.4.tar.bz2
      rm -f qwt-6.1.4.tar.bz2
    fi
    if [ ! -d /usr/local/include/qwt ]
    then
      cd $SOURCE_DIR/qwt-6.1.4
      sed -i -e's@QWT_INSTALL_PREFIX    = /usr/local/qwt-$$QWT_VERSION@QWT_INSTALL_PREFIX = /usr/local@' qwtconfig.pri
      sed -i -e's@QWT_INSTALL_HEADERS   = $${QWT_INSTALL_PREFIX}/include@QWT_INSTALL_HEADERS = $${QWT_INSTALL_PREFIX}/include/qwt@' qwtconfig.pri
      qmake
      make -j${BUILD_CORE}
      sudo make install
    fi
  fi
}
