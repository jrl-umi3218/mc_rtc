export ROS_DISTRO=
export PIP_DEPENDENCIES="Cython coverage nose numpy matplotlib pyqt5"
export WITH_ROS_SUPPORT="false"
export INSTALL_PREFIX=/c/devel/install

build_system_dependency()
{
  REPO=$1
  REF=$2
  SRC=$3
  if [ ! -f $SRC/build/install_manifest.txt ]
  then
    if [ ! -d $SRC ]
    then
      git clone --recursive https://github.com/$REPO $SRC
      cd $SRC
      git checkout $REF
    fi
    if $CLONE_ONLY
    then
      return
    fi
    mkdir -p $SRC/build
    cd $SRC/build
    cmake ../ -DCMAKE_BUILD_TYPE=$BUILD_TYPE         \
              -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
              -DBUILD_TESTING:BOOL=OFF               \
              ${@:3}
    cmake --build . --config ${BUILD_TYPE}
    if [ $? -ne 0 ]
    then
      echo "Build failed for $1"
      exit 1
    fi
    ${SUDO_CMD} cmake --build . --target install --config ${BUILD_TYPE}
    if [ $? -ne 0 ]
    then
      echo "Installation failed for $1"
      exit 1
    fi
  fi
}

mc_rtc_extra_steps()
{
  build_system_dependency eigenteam/eigen-git-mirror 3.3.7 "$SOURCE_DIR/eigen"
  build_system_dependency leethomason/tinyxml2 7.1.0 "$SOURCE_DIR/tinyxml2"
  build_system_dependency libgeos/geos 3.8.1 "$SOURCE_DIR/geos" -DBUILD_TESTING:BOOL=ON
  build_system_dependency nanomsg/nanomsg 1.1.5 "$SOURCE_DIR/nanomsg"
  build_system_dependency jbeder/yaml-cpp 29dcf92f870ee51cce8d68f8fcfe228942e8dfe1 "$SOURCE_DIR/yaml-cpp" -DYAML_CPP_BUILD_TESTS:BOOL=OFF
  build_system_dependency google/benchmark master "$SOURCE_DIR/benchmark" -DBENCHMARK_ENABLE_GTEST_TESTS:BOOL=OFF
}
