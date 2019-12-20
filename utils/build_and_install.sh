#!/bin/bash

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

shopt -s expand_aliases

##########################
#  --  Configuration --  #
##########################

readonly this_dir=`cd $(dirname $0); pwd`
readonly mc_rtc_dir=`cd $this_dir/..; pwd`
readonly SOURCE_DIR=`cd $mc_rtc_dir/../; pwd`

readonly PYTHON_VERSION=`python -c 'import sys; print("{}.{}".format(sys.version_info.major, sys.version_info.minor))'`

#default settings
INSTALL_PREFIX="/usr/local"
WITH_ROS_SUPPORT="true"
WITH_PYTHON_SUPPORT="true"
PYTHON_USER_INSTALL="false"
PYTHON_FORCE_PYTHON2="false"
PYTHON_FORCE_PYTHON3="false"
PYTHON_BUILD_PYTHON2_AND_PYTHON3="false"
WITH_HRP2="false"
WITH_HRP4="false"
BUILD_TYPE="RelWithDebInfo"
BUILD_TESTING="true"
INSTALL_APT_DEPENDENCIES="true"
if command -v nproc > /dev/null
then
   BUILD_CORE=`nproc`
else
   BUILD_CORE=`sysctl -n hw.ncpu`
fi
export CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_CORE}

mc_rtc_extra_steps()
{
  true
}

if [[ $OSTYPE == "darwin"* ]]
then
  . $this_dir/config_build_and_install.macos.sh
elif [[ $OSTYPE == "linux-gnu" ]]
then
  if [ -f $this_dir/config_build_and_install.`lsb_release -sc`.sh ]
  then
    . $this_dir/config_build_and_install.`lsb_release -sc`.sh
  else
    ROS_DISTRO=""
    APT_DEPENDENCIES=""
    ROS_APT_DEPENDENCIES=""
  fi
else
  # Assume Windows
  . $this_dir/config_build_and_install.windows.sh
fi

readonly HELP_STRING="$(basename $0) [OPTIONS] ...
    --help                     (-h)               : print this help
    --install-prefix           (-i) PATH          : the directory used to install everything         (default $INSTALL_PREFIX)
    --build-type                    Type          : the build type to use                            (default $BUILD_TYPE)
    --build-testing                 {true, false} : whether to build and run unit tests              (default $BUILD_TESTING)
    --build-core               (-j) N             : number of cores used for building                (default $BUILD_CORE)
    --with-hrp2                                   : enable HRP2 (requires mc-hrp2 group access)      (default $WITH_HRP2)
    --with-hrp4                                   : enable HRP4 (requires mc-hrp4 group access)      (default $WITH_HRP4)
    --with-python-support           {true, false} : whether to build with Python support             (default $WITH_PYTHON_SUPPORT)
    --python-user-install           {true, false} : whether to install Python bindings with user     (default $PYTHON_USER_INSTALL)
    --python-force-python2          {true, false} : whether to enforce the use of Python 2           (default $PYTHON_FORCE_PYTHON2)
    --python-force-python3          {true, false} : whether to enforce the use of Python 3           (default $PYTHON_FORCE_PYTHON3)
    --python-build-2-and-3          {true, false} : whether to build both Python 2 and Python 3      (default $PYTHON_BUILD_PYTHON2_AND_PYTHON3)
    --with-ros-support              {true, false} : whether to build with ROS support                (default $WITH_ROS_SUPPORT)
    --ros-distro                    NAME          : the ros distro to use                            (default $ROS_DISTRO)
    --install-system-dependencies      {true, false} : whether to install system packages            (default $INSTALL_APT_DEPENDENCIES)
"

#helper for parsing
check_true_false()
{
    if [ "true" != "$2" ] && [ "false" != "$2" ]
    then
        echo "passed parameter '$2' as flag for '$1'. the parameter has to be 'true' or 'false'"
        exit 1
    fi
}
#parse arguments
i=1
while [[ $# -ge $i ]]
do
    key="${!i}"
    case $key in
        -h|--help)
        echo "$HELP_STRING"
        exit
        ;;

        -i|--install-prefix)
        i=$(($i+1))
        INSTALL_PREFIX="${!i}"
        ;;

        --with-ros-support)
        i=$(($i+1))
        WITH_ROS_SUPPORT="${!i}"
        check_true_false --with-ros-support "$WITH_ROS_SUPPORT"
        ;;

        --with-python-support)
        i=$(($i+1))
        WITH_PYTHON_SUPPORT="${!i}"
        check_true_false --with-python-support "$WITH_PYTHON_SUPPORT"
        ;;

        --python-user-install)
        i=$(($i+1))
        PYTHON_USER_INSTALL="${!i}"
        check_true_false --python-user-install "$PYTHON_USER_INSTALL"
        ;;

        --python-force-python2)
        i=$(($i+1))
        PYTHON_FORCE_PYTHON2="${!i}"
        check_true_false --python-force-python2 "$PYTHON_FORCE_PYTHON2"
        ;;

        --python-force-python3)
        i=$(($i+1))
        PYTHON_FORCE_PYTHON3="${!i}"
        check_true_false --python-force-python3 "$PYTHON_FORCE_PYTHON3"
        ;;

        --python-build-2-and-3)
        i=$(($i+1))
        PYTHON_BUILD_PYTHON2_AND_PYTHON3="${!i}"
        check_true_false --python-build-2-and-3 "$PYTHON_BUILD_PYTHON2_AND_PYTHON3"
        ;;

        --with-hrp2)
          i=$(($i+1))
          WITH_HRP2="${!i}"
          check_true_false --with-hrp2 "$WITH_HRP2"
          ;;

        --with-hrp4)
        i=$(($i+1))
        WITH_HRP4="${!i}"
        check_true_false --with-hrp4 "$WITH_HRP4"
        ;;

        --build-type)
        i=$(($i+1))
        BUILD_TYPE="${!i}"
        ;;

        --build-testing)
        i=$(($i+1))
        BUILD_TESTING="${!i}"
        check_true_false --build-testing "$BUILD_TESTING"
        ;;

        --install-apt-dependencies)
        i=$(($i+1))
        INSTALL_APT_DEPENDENCIES="${!i}"
        check_true_false --install-apt-dependencies "$INSTALL_APT_DEPENDENCIES"
        ;;

        -j|--build-core)
        i=$(($i+1))
        BUILD_CORE="${!i}"
        ;;

        --ros-distro)
        i=$(($i+1))
        ROS_DISTRO="${!i}"
        ;;

        *)
        echo "unknown parameter $i ($key)"
        exit 1
        ;;
    esac

    i=$(($i+1))
done
if $WITH_PYTHON_SUPPORT
then
  WITH_PYTHON_SUPPORT=ON
else
  WITH_PYTHON_SUPPORT=OFF
fi
if $PYTHON_USER_INSTALL
then
  PYTHON_USER_INSTALL=ON
else
  PYTHON_USER_INSTALL=OFF
fi
if $PYTHON_FORCE_PYTHON2
then
  PYTHON_FORCE_PYTHON2=ON
else
  PYTHON_FORCE_PYTHON2=OFF
fi
if $PYTHON_FORCE_PYTHON3
then
  PYTHON_FORCE_PYTHON3=ON
else
  PYTHON_FORCE_PYTHON3=OFF
fi
if $PYTHON_BUILD_PYTHON2_AND_PYTHON3
then
  PYTHON_BUILD_PYTHON2_AND_PYTHON3=ON
else
  PYTHON_BUILD_PYTHON2_AND_PYTHON3=OFF
fi
#make settings readonly
readonly INSTALL_PREFIX
readonly WITH_ROS_SUPPORT
readonly WITH_PYTHON_SUPPORT
readonly WITH_PYTHON_SUPPORT
readonly PYTHON_FORCE_PYTHON2
readonly PYTHON_FORCE_PYTHON3
readonly PYTHON_BUILD_PYTHON2_AND_PYTHON3
readonly BUILD_TYPE
readonly INSTALL_APT_DEPENDENCIES
readonly BUILD_CORE
readonly BUILD_TESTING

readonly ROS_APT_DEPENDENCIES="ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rviz"

alias git_clone="git clone --recursive"
alias git_update="git pull && git submodule sync && git submodule update"

SUDO_CMD='sudo -E'
if [ ! -d $INSTALL_PREFIX ]
then
  mkdir -p $INSTALL_PREFIX
fi
if [ -w $INSTALL_PREFIX ]
then
  SUDO_CMD=
  PYTHON_USER_INSTALL=ON
fi

export PATH=$INSTALL_PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$DYLD_LIBRARY_PATH
export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages:$PYTHONPATH

##############################
#  --  APT/Brew dependencies  --  #
##############################
if [[ $OSTYPE == "darwin"* ]]
then
  export OS=macOS
  # Install brew on the system
  if $INSTALL_APT_DEPENDENCIES
  then
    if ! command -v brew
    then
      /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    fi
    brew update
    brew cask install $CASK_DEPENDENCIES
    brew install $BREW_DEPENDENCIES
    if [ "x$WITH_PYTHON_SUPPORT" == xON ]
    then
      if [ "x$PYTHON_BUILD_PYTHON2_AND_PYTHON3" == xON ]
      then
        sudo pip2 install $PIP_DEPENDENCIES
        sudo pip3 install $PIP_DEPENDENCIES
      elif [ "x$PYTHON_FORCE_PYTHON2" == xON ]
      then
        sudo pip2 install $PIP_DEPENDENCIES
      elif [ "x$PYTHON_FORCE_PYTHON3" == xON ]
      then
        sudo pip3 install $PIP_DEPENDENCIES
      else
        sudo pip install $PIP_DEPENDENCIES
      fi
      mc_rtc_extra_steps
    fi
  else
    echo "Skip installation of system dependencies"
  fi
elif [[ $OSTYPE == "linux-gnu" ]]
then
  export OS=$(lsb_release -si)
  if [ $OS = Ubuntu ]
  then
    if $INSTALL_APT_DEPENDENCIES
    then
      sudo apt-get update
      sudo apt-get -y install ${APT_DEPENDENCIES}
      mc_rtc_extra_steps
    else
      echo "Skip installation of system dependencies"
    fi
  else
    echo "This script does not support your OS: ${OS}, assuming you have installed the required system dependencies already"
  fi
else
  export OS=Windows
  if [ "x$WITH_PYTHON_SUPPORT" == xON ]
  then
    pip install ${PIP_DEPENDENCIES}
  fi
  mc_rtc_extra_steps
fi

git_dependency_parsing()
{
  _input=$1
  git_dep=${_input%%#*}
  git_dep_branch=${_input##*#}
  if [ "$git_dep_branch" = "$git_dep" ]; then
    if [ -e "$2" ]; then
      git_dep_branch=$2
    else
      git_dep_branch="master"
    fi
  fi
  git_dep_uri_base=${git_dep%%:*}
  if [ "$git_dep_uri_base" = "$git_dep" ]; then
    git_dep_uri="https://github.com/$git_dep"
  else
    git_dep_uri=$git_dep
    git_dep=${git_dep##*:}
  fi
  git_dep=`basename $git_dep`
}

clone_git_dependency()
{
  git_dependency_parsing $1
  cd "$2"
  mkdir -p "$git_dep"
  if [ ! -d "$git_dep/.git" ]
  then
    git_clone -b $git_dep_branch "$git_dep_uri" "$git_dep"
  else
    pushd .
    cd "$git_dep"
    git_update
    popd
  fi
}

# This function is used in Windows to hide sh from the PATH
hide_sh()
{
  export OLD_PATH=${PATH}
  echo "PATH was ${OLD_PATH}"
  sh_path=`which sh || echo ""`
  while [[ "$sh_path" != "" ]]
  do
    sh_dir=`dirname $sh_path`
    export PATH=`echo $PATH|sed -e "s@:${sh_dir}@@"`
    sh_path=`which sh || echo ""`
  done
  echo "PATH is ${PATH}"
}

restore_path()
{
  export PATH=${OLD_PATH}
}

build_project()
{
  cmake --build . --config ${BUILD_TYPE}
  if [ $? -ne 0 ]
  then
    echo "Build failed for $1"
    exit 1
  fi
  if [ -f install_manifest.txt ]
  then
    ${SUDO_CMD} cmake --build . --target uninstall --config ${BUILD_TYPE}
  fi
  ${SUDO_CMD} cmake --build . --target install --config ${BUILD_TYPE}
  if [ $? -ne 0 ]
  then
    echo "Installation failed for $1"
    exit 1
  fi
}

build_git_dependency_configure_and_build()
{
  clone_git_dependency $1 "$SOURCE_DIR"
  echo "--> Compiling $git_dep (branch $git_dep_branch)"
  mkdir -p "$SOURCE_DIR/$git_dep/build"
  cd "$SOURCE_DIR/$git_dep/build"
  if [[ $OS == "Windows" ]]
  then
    hide_sh
  fi
  cmake .. -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
           -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
           -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
           -DPYTHON_BINDING_FORCE_PYTHON2:BOOL=${PYTHON_FORCE_PYTHON2} \
           -DPYTHON_BINDING_FORCE_PYTHON3:BOOL=${PYTHON_FORCE_PYTHON3} \
           -DPYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON3:BOOL=${PYTHON_BUILD_PYTHON2_AND_PYTHON3} \
           -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
           -DBUILD_PYTHON_INTERFACE:BOOL=OFF \
           ${CMAKE_ADDITIONAL_OPTIONS}
  if [ $? -ne 0 ]
  then
    echo "CMake configuration failed for $git_dep"
    exit 1
  fi
  build_project $git_dep
  if [[ $OS == "Windows" ]]
  then
    restore_path
  fi
}

build_git_dependency()
{
  if $BUILD_TESTING
  then
    build_git_dependency_configure_and_build $1
    ctest -C ${BUILD_TYPE}
    if [ $? -ne 0 ]
    then
      echo "Testing failed for $git_dep"
      exit 1
    fi
  else
    build_git_dependency_no_test $1
  fi
}

build_git_dependency_no_test()
{
  OLD_CMAKE_OPTIONS="${CMAKE_ADDITIONAL_OPTIONS}"
  export CMAKE_ADDITIONAL_OPTIONS="${OLD_CMAKE_OPTIONS} -DBUILD_TESTING:BOOL=OFF"
  build_git_dependency_configure_and_build $1
  export CMAKE_ADDITIONAL_OPTIONS="${OLD_CMAKE_OPTIONS}"
}

build_catkin_git_dependency()
{
  clone_git_dependency $1 "$2/src"
  echo "--> Compiling $git_dep (branch $git_dep_branch)"
  cd $2
  catkin_make || (echo "catkin build failed for $git_dep" && exit 1)
}

########################
##  -- Install ROS --  #
########################

if $WITH_ROS_SUPPORT
then
  if [ ! -e /opt/ros/${ROS_DISTRO}/setup.bash ]
  then
    if [ $OS = Ubuntu ]
    then
      sudo mkdir -p /etc/apt/sources.list.d/
      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -c -s` main" > /etc/apt/sources.list.d/ros-latest.list'
      wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
      sudo apt-get update
      sudo apt-get install -y ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-rosdoc-lite python-catkin-lint ${ROS_APT_DEPENDENCIES}
    else
      echo "Please install ROS and the required dependencies (${ROS_APT_DEPENDENCIES}) before continuing your installation or disable ROS support"
      exit 1
    fi
  fi
  . /opt/ros/${ROS_DISTRO}/setup.bash
  CATKIN_DATA_WORKSPACE=$SOURCE_DIR/catkin_data_ws
  CATKIN_DATA_WORKSPACE_SRC=${CATKIN_DATA_WORKSPACE}/src/
  mkdir -p ${CATKIN_DATA_WORKSPACE_SRC}
  cd ${CATKIN_DATA_WORKSPACE_SRC}
  catkin_init_workspace || true
  cd ${CATKIN_DATA_WORKSPACE}
  catkin_make
  . $CATKIN_DATA_WORKSPACE/devel/setup.bash
  CATKIN_WORKSPACE=$SOURCE_DIR/catkin_ws
  CATKIN_WORKSPACE_SRC=${CATKIN_WORKSPACE}/src/
  mkdir -p ${CATKIN_WORKSPACE_SRC}
  cd ${CATKIN_WORKSPACE_SRC}
  catkin_init_workspace || true
  cd ${CATKIN_WORKSPACE}
  catkin_make
  . $CATKIN_WORKSPACE/devel/setup.bash
fi

###############################
##  --  GIT dependencies  --  #
###############################

build_git_dependency_no_test humanoid-path-planner/hpp-spline#v4.7.0
if [ "x$WITH_PYTHON_SUPPORT" == xON ]
then
  build_git_dependency jrl-umi3218/Eigen3ToPython
fi
build_git_dependency jrl-umi3218/SpaceVecAlg
build_git_dependency jrl-umi3218/sch-core
if [ "x$WITH_PYTHON_SUPPORT" == xON ]
then
  build_git_dependency jrl-umi3218/sch-core-python
fi
build_git_dependency jrl-umi3218/RBDyn
build_git_dependency jrl-umi3218/eigen-qld
build_git_dependency jrl-umi3218/Tasks
build_git_dependency jrl-umi3218/mc_rbdyn_urdf
if $WITH_ROS_SUPPORT
then
  build_catkin_git_dependency jrl-umi3218/mc_rtc_data $CATKIN_DATA_WORKSPACE
  build_catkin_git_dependency jrl-umi3218/mc_rtc_msgs $CATKIN_DATA_WORKSPACE
else
  build_git_dependency jrl-umi3218/mc_rtc_data
fi

##########################
#  --  Build mc_rtc  --  #
##########################
cd $mc_rtc_dir
git submodule sync || true
git submodule update --init
mkdir -p build
cd build
if $BUILD_TESTING
then
  BUILD_TESTING_OPTION=ON
else
  BUILD_TESTING_OPTION=OFF
fi
if $WITH_ROS_SUPPORT
then
  cmake ../ -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
            -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
            -DBUILD_TESTING:BOOL=${BUILD_TESTING_OPTION} \
            -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
            -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
            -DPYTHON_BINDING_FORCE_PYTHON2:BOOL=${PYTHON_FORCE_PYTHON2} \
            -DPYTHON_BINDING_FORCE_PYTHON3:BOOL=${PYTHON_FORCE_PYTHON3} \
            -DPYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON3:BOOL=${PYTHON_BUILD_PYTHON2_AND_PYTHON3} \
            ${CMAKE_ADDITIONAL_OPTIONS}
else
  cmake ../ -DCMAKE_BUILD_TYPE:STRING="'$BUILD_TYPE'" \
            -DCMAKE_INSTALL_PREFIX:STRING="'$INSTALL_PREFIX'" \
            -DBUILD_TESTING:BOOL=${BUILD_TESTING_OPTION} \
            -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
            -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
            -DPYTHON_BINDING_FORCE_PYTHON2:BOOL=${PYTHON_FORCE_PYTHON2} \
            -DPYTHON_BINDING_FORCE_PYTHON3:BOOL=${PYTHON_FORCE_PYTHON3} \
            -DPYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON3:BOOL=${PYTHON_BUILD_PYTHON2_AND_PYTHON3} \
            ${CMAKE_ADDITIONAL_OPTIONS} \
            -DDISABLE_ROS=ON
fi
if [ $? -ne 0 ]
then
  echo "CMake configuration failed for mc_rtc"
  exit 1
fi
build_project mc_rtc
if $BUILD_TESTING
then
  ctest -C ${BUILD_TYPE}
fi
if [ $? -ne 0 ]
then
  if [ "x$WITH_PYTHON_SUPPORT" == xON ]
  then
    echo "mc_rtc testing failed, asssuming you need to rebuild your Python bindings"
    if [ "x$PYTHON_BUILD_PYTHON2_AND_PYTHON3" == xON ]
    then
      make force-mc_rtc-python2-bindings
      make force-mc_rtc-python3-bindings
    elif [ "x$PYTHON_FORCE_PYTHON2" == xON ]
    then
      make force-mc_rtc-python2-bindings
    elif [ "x$PYTHON_FORCE_PYTHON3" == xON ]
    then
      make force-mc_rtc-python3-bindings
    else
      make force-mc_rtc-python-bindings
    fi
    ${SUDO_CMD} make install
    make test
    if [ $? -ne 0 ]
    then
      echo "mc_rtc is still failing"
      exit 1
    fi
  else
    echo "Testing failed for mc_rtc"
    exit 1
  fi
fi

##############################
#  --  Build mc_rtc_ros  --  #
##############################
if $WITH_ROS_SUPPORT
then
  build_catkin_git_dependency jrl-umi3218/mc_rtc_ros $CATKIN_WORKSPACE
fi

################################
#  --  Build extra modules  -- #
################################
if $WITH_HRP2
then
  if $WITH_ROS_SUPPORT
  then
    build_catkin_git_dependency git@gite.lirmm.fr:mc-hrp2/hrp2_drc $CATKIN_DATA_WORKSPACE
  else
    build_git_dependency git@gite.lirmm.fr:mc-hrp2/hrp2_drc
  fi
  build_git_dependency git@gite.lirmm.fr:mc-hrp2/mc-hrp2.git
fi

if $WITH_HRP4
then
  if $WITH_ROS_SUPPORT
  then
    build_catkin_git_dependency git@gite.lirmm.fr:mc-hrp4/hrp4 $CATKIN_DATA_WORKSPACE
  else
    build_git_dependency git@gite.lirmm.fr:mc-hrp4/hrp4
  fi
  build_git_dependency git@gite.lirmm.fr:mc-hrp4/mc-hrp4.git
fi

echo "Installation finished, please add the following lines to your .bashrc/.zshrc"
echo ""
if [ ${OS} = Darwin ]
then
  echo """
  export PATH=$INSTALL_PREFIX/bin:\$PATH
  export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$DYLD_LIBRARY_PATH
  export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH
  export PYTHONPATH=$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages:\$PYTHONPATH
  """
else
  echo """
  export PATH=$INSTALL_PREFIX/bin:\$PATH
  export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$LD_LIBRARY_PATH
  export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH
  export PYTHONPATH=$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages:\$PYTHONPATH
  """
  if $WITH_ROS_SUPPORT
  then
    echo "source $CATKIN_WORKSPACE/devel/setup.bash"
    echo ""
    echo "If you are running zsh, replace setup.bash with setup.zsh in that last line"
  fi
fi
