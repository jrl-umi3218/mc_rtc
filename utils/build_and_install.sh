#!/bin/bash

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

shopt -s expand_aliases

##########################
#  --  Configuration --  #
##########################

readonly mc_rtc_dir=`cd $(dirname $0)/..; pwd`

readonly SOURCE_DIR=`cd $mc_rtc_dir/../; pwd`

readonly PYTHON_VERSION=`python -c 'import sys; print("{}.{}".format(sys.version_info.major, sys.version_info.minor))'`

#default settings
INSTALL_PREFIX="/usr/local"
WITH_ROS_SUPPORT="true"
WITH_VREP_SUPPORT="true"
WITH_PYTHON_SUPPORT="true"
PYTHON_USER_INSTALL="false"
PYTHON_FORCE_PYTHON2="false"
PYTHON_FORCE_PYTHON3="false"
PYTHON_BUILD_PYTHON2_AND_PYTHON3="false"
WITH_HRP2="false"
WITH_HRP4="false"
WITH_VREP="true"
VREP_PATH=
BUILD_TYPE="RelWithDebInfo"
INSTALL_APT_DEPENDENCIES="true"
if command -v nproc > /dev/null
then
   BUILD_CORE=`nproc`
else
   BUILD_CORE=`sysctl -n hw.ncpu`
fi

if [ `lsb_release -sc` = "trusty" ]
then
  ROS_DISTRO=indigo
elif [ `lsb_release -sc` = "xenial" ]
then
  ROS_DISTRO=kinetic
else
  ROS_DISTRO=melodic
fi

readonly HELP_STRING="$(basename $0) [OPTIONS] ...
    --help                     (-h)               : print this help
    --install-prefix           (-i) PATH          : the directory used to install everything         (default $INSTALL_PREFIX)
    --build-type                    Type          : the build type to use                            (default $BUILD_TYPE)
    --build-core               (-j) N             : number of cores used for building                (default $BUILD_CORE)
    --with-hrp2                                   : enable HRP2 (requires mc-hrp2 group access)      (default $WITH_HRP2)
    --with-hrp4                                   : enable HRP4 (requires mc-hrp4 group access)      (default $WITH_HRP4)
    --with-python-support           {true, false} : whether to build with Python support             (default $WITH_PYTHON_SUPPORT)
    --python-user-install           {true, false} : whether to install Python bindings with user     (default $PYTHON_USER_INSTALL)
    --python-force-python2          {true, false} : whether to enforce the use of Python 2           (default $PYTHON_FORCE_PYTHON2)
    --python-force-python3          {true, false} : whether to enforce the use of Python 3           (default $PYTHON_FORCE_PYTHON3)
    --python-build-2-and-3          {true, false} : whether to build both Python 2 and Python 3      (default $PYTHON_BUILD_PYTHON2_AND_PYTHON3)
    --with-ros-support              {true, false} : whether to build with ROS support                (default $WITH_ROS_SUPPORT)
    --with-vrep-support             {true, false} : whether to build with VREP support               (default $WITH_VREP_SUPPORT)
    --ros-distro                    NAME          : the ros distro to use                            (default $ROS_DISTRO)
    --install-apt-dependencies      {true, false} : whether to install packages                      (default $INSTALL_APT_DEPENDENCIES)
    --vrep-path                     PATH          : where to find vrep (will be downloaded if empty) (default $VREP_PATH)
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

        --with-vrep-support)
        i=$(($i+1))
        WITH_VREP_SUPPORT="${!i}"
        check_true_false --with-vrep-support "$WITH_VREP_SUPPORT"
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

        --vrep-path)
        i=$(($i+1))
        VREP_PATH="${!i}"
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
readonly WITH_VREP_SUPPORT
readonly WITH_PYTHON_SUPPORT
readonly WITH_PYTHON_SUPPORT
readonly PYTHON_USER_INSTALL
readonly PYTHON_FORCE_PYTHON2
readonly PYTHON_FORCE_PYTHON3
readonly PYTHON_BUILD_PYTHON2_AND_PYTHON3
readonly BUILD_TYPE
readonly INSTALL_APT_DEPENDENCIES
readonly BUILD_CORE

readonly ROS_APT_DEPENDENCIES="ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rviz"

alias git_clone="git clone --recursive"
alias git_update="git pull && git submodule update"

SUDO_CMD='sudo -E'
PIP_USER=
if [ ! -d $INSTALL_PREFIX ]
then
  mkdir -p $INSTALL_PREFIX
fi
if [ -w $INSTALL_PREFIX ]
then
  SUDO_CMD=
  PIP_USER='--user'
fi

readonly gitlab_ci_yml=$mc_rtc_dir/.gitlab-ci.yml

export PATH=$INSTALL_PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$DYLD_LIBRARY_PATH
export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages:$PYTHONPATH

yaml_to_env()
{
  local var=$1
  local f=$2
  tmp=`grep "$var:" $f|sed -e"s/.*${var}: \"\(.*\)\"/\1/"`
  export $var="$tmp"
}

##############################
#  --  APT/Brew dependencies  --  #
##############################
KERN=$(uname -s)
if [ $KERN = Darwin ]
then
  export OS=Darwin
  # Install brew on the system
  if $INSTALL_APT_DEPENDENCIES
  then
    if ! command -v brew
    then
      /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    fi
    brew update
    brew install coreutils pkg-config gnu-sed wget gcc python cmake doxygen libtool tinyxml2 geos boost eigen || true
    # eigen3.pc is broken in brew release
    gsed -i -e's@Cflags: -Iinclude/eigen3@Cflags: -I/usr/local/include/eigen3@' /usr/local/lib/pkgconfig/eigen3.pc
  else
    echo "SKIPPING INSTALLATION OF BREW DEPENDENCIES"
  fi
else
  export OS=$(lsb_release -si)
  export UBUNTU_MAJOR=0
  if [ $OS = Ubuntu ]
  then
    UBUNTU_VERSION=`lsb_release -rs`
    export UBUNTU_MAJOR=${UBUNTU_VERSION%%.*}
    yaml_to_env "APT_DEPENDENCIES" $gitlab_ci_yml
    APT_DEPENDENCIES=`echo $APT_DEPENDENCIES|sed -e's/libspacevecalg-dev//'|sed -e's/librbdyn-dev//'|sed -e's/libeigen-qld-dev//'|sed -e's/libsch-core-dev//'|sed -e's/libtasks-qld-dev//'|sed -e's/libmc-rbdyn-urdf-dev//'|sed -e's/python-tasks//'|sed -e's/python-mc-rbdyn-urdf//'`
    APT_DEPENDENCIES="cmake build-essential gfortran doxygen libeigen3-dev python-pip python3-pip wget cython3 python3-numpy python3-nose python3-coverage python-git python-pyside $APT_DEPENDENCIES"
    if [ $UBUNTU_MAJOR -ge 16 ]
    then
      APT_DEPENDENCIES="libyaml-cpp-dev $APT_DEPENDENCIES"
    fi
    if $INSTALL_APT_DEPENDENCIES
    then
        sudo apt-get update
        sudo apt-get -y install ${APT_DEPENDENCIES}
    else
        echo "SKIPPING INSTALLATION OF APT_DEPENDENCIES ($APT_DEPENDENCIES)"
    fi
  else
    echo "This script does not support your OS: ${OS}, please contact the maintainer"
    exit 1
  fi
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
    git_dep_uri="git://github.com/$git_dep"
  else
    git_dep_uri=$git_dep
    git_dep=${git_dep##*:}
  fi
  git_dep=`basename $git_dep`
}

build_git_dependency_configure_and_build()
{
  git_dependency_parsing $1
  echo "--> Compiling $git_dep (branch $git_dep_branch)"
  cd "$SOURCE_DIR"
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
  mkdir -p $git_dep/build
  cd "$git_dep/build"
  cmake .. -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
           -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
           -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
           -DPYTHON_BINDING_FORCE_PYTHON2:BOOL=${PYTHON_FORCE_PYTHON2} \
           -DPYTHON_BINDING_FORCE_PYTHON3:BOOL=${PYTHON_FORCE_PYTHON3} \
           -DPYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON3:BOOL=${PYTHON_BUILD_PYTHON2_AND_PYTHON3} \
           -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
           -DVREP_PATH:STRING="$VREP_PATH" \
           -DBUILD_PYTHON_INTERFACE:BOOL=OFF \
           -DYAML_BUILD_SHARED_LIBS:BOOL=ON \
           ${CMAKE_ADDITIONAL_OPTIONS}
  make -j${BUILD_CORE} || exit 1
}

build_git_dependency()
{
  build_git_dependency_configure_and_build $1
  make test || exit 1
  ${SUDO_CMD} make install || exit 1
}

build_git_dependency_no_test()
{
  build_git_dependency_configure_and_build $1
  ${SUDO_CMD} make install || exit 1
}
###############################
##  --  GIT dependencies  --  #
###############################
yaml_to_env "GIT_DEPENDENCIES" $gitlab_ci_yml
# Add some source dependencies
if [ "x$WITH_PYTHON_SUPPORT" == xON ]
then
  GIT_DEPENDENCIES="jrl-umi3218/Eigen3ToPython jrl-umi3218/SpaceVecAlg jrl-umi3218/RBDyn jrl-umi3218/eigen-qld jrl-umi3218/sch-core jrl-umi3218/sch-core-python jrl-umi3218/mc_rbdyn_urdf ${GIT_DEPENDENCIES}"
else
  GIT_DEPENDENCIES="jrl-umi3218/SpaceVecAlg jrl-umi3218/RBDyn jrl-umi3218/eigen-qld jrl-umi3218/sch-core jrl-umi3218/mc_rbdyn_urdf ${GIT_DEPENDENCIES}"
fi
if [ $UBUNTU_MAJOR -ge 16 ]
then
  GIT_DEPENDENCIES=`echo $GIT_DEPENDENCIES|sed -e's@jbeder/yaml-cpp@@'`
fi
if $WITH_ROS_SUPPORT
then
  ROS_GIT_DEPENDENCIES="git@gite.lirmm.fr:multi-contact/mc_rtc_ros_data#master"
  if $WITH_HRP2
  then
    ROS_GIT_DEPENDENCIES="$ROS_GIT_DEPENDENCIES git@gite.lirmm.fr:mc-hrp2/hrp2_drc"
  fi
  if $WITH_HRP4
  then
    ROS_GIT_DEPENDENCIES="$ROS_GIT_DEPENDENCIES git@gite.lirmm.fr:mc-hrp4/hrp4"
  fi
else
  ROS_GIT_DEPENDENCIES=""
  GIT_DEPENDENCIES="$GIT_DEPENDENCIES git@gite.lirmm.fr:multi-contact/mc_rtc_ros_data"
  if $WITH_HRP2
  then
    GIT_DEPENDENCIES="$GIT_DEPENDENCIES git@gite.lirmm.fr:mc-hrp2/hrp2_drc"
  fi
  if $WITH_HRP4
  then
    GIT_DEPENDENCIES="$GIT_DEPENDENCIES git@gite.lirmm.fr:mc-hrp4/hrp4"
  fi
fi
for package in ${GIT_DEPENDENCIES}; do
  build_git_dependency "$package"
done

################################
#  -- Handle ROS packages  --  #
################################
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
  CATKIN_SRC_DIR=$SOURCE_DIR/catkin_ws/src
  mkdir -p $CATKIN_SRC_DIR
  cd $CATKIN_SRC_DIR
  catkin_init_workspace || true
  for package in ${ROS_GIT_DEPENDENCIES}; do
    git_dependency_parsing $package
    cd $CATKIN_SRC_DIR
    if [ ! -d "$git_dep/.git" ]
    then
      git_clone -b $git_dep_branch "$git_dep_uri" "$git_dep"
    else
      cd "$git_dep"
      git_update
    fi
  done
  cd $SOURCE_DIR/catkin_ws
  catkin_make || exit 1
  . $SOURCE_DIR/catkin_ws/devel/setup.bash
fi

##########################
#  --  Build mc_rtc  --  #
##########################
cd $mc_rtc_dir
git submodule update --init
mkdir -p build
cd build
if $WITH_ROS_SUPPORT
then
  cmake ../ -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
            -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
            -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
            -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
            ${CMAKE_ADDITIONAL_OPTIONS}
else
  cmake ../ -DCMAKE_BUILD_TYPE:STRING="'$BUILD_TYPE'" \
            -DCMAKE_INSTALL_PREFIX:STRING="'$INSTALL_PREFIX'" \
            -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
            -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
            ${CMAKE_ADDITIONAL_OPTIONS} \
            -DDISABLE_ROS=ON
fi
make -j$BUILD_CORE || exit 1
${SUDO_CMD} make install

##############################
#  --  Build mc_rtc_ros  --  #
##############################
if $WITH_ROS_SUPPORT
then
  CATKIN_DIR=$SOURCE_DIR/catkin_ws
  cd $CATKIN_DIR/src
  if [ ! -d mc_rtc_ros/.git ]
  then
    git_clone git@gite.lirmm.fr:multi-contact/mc_rtc_ros
  else
    cd mc_rtc_ros
    git_update
  fi
  cd $CATKIN_DIR
  catkin_make || exit 1
  . $CATKIN_DIR/devel/setup.bash
fi

################################
#  --  Build extra modules  -- #
################################
if $WITH_HRP2
then
  build_git_dependency git@gite.lirmm.fr:mc-hrp2/mc-hrp2.git
fi

if $WITH_HRP4
then
  build_git_dependency git@gite.lirmm.fr:mc-hrp4/mc-hrp4.git
fi

####################################################
#  -- Setup VREP, vrep-api-wrapper and mc_vrep --  #
####################################################
if $WITH_VREP_SUPPORT
then
  if [ -z "${VREP_PATH}" ]
  then
    VREP_MAJOR="V-REP_PRO_EDU_V3_4_0"
    cd $SOURCE_DIR
    if [ $OS = Darwin ]
    then
      VREP_MACOS="${VREP_MAJOR}_Mac"
      if [ ! -d $VREP_MACOS ]
      then
        wget http://coppeliarobotics.com/files/${VREP_MACOS}.zip
        unzip ${VREP_MACOS}.zip
      fi
      VREP_PATH=$SOURCE_DIR/$VREP_MACOS
    else
      if [ "`uname -i`" != "x86_64" ]
      then
        VREP_MAJOR="V-REP_PRO_EDU_V3_3_2"
        echo "[WARNING] VREP support for 32 bits stopped after 3.3.2, it might not work properly with the models or softwares we provide"
      fi
      VREP_LINUX="${VREP_MAJOR}_Linux"
      if [ ! -d ${VREP_LINUX} ]
      then
        wget http://coppeliarobotics.com/files/${VREP_LINUX}.tar.gz
        tar xzf ${VREP_LINUX}.tar.gz
      fi
      VREP_PATH=$SOURCE_DIR/$VREP_LINUX
    fi
  fi
  [ ! -e "$SOURCE_DIR/vrep" ] && ln -s "$VREP_PATH" "$SOURCE_DIR/vrep"

  build_git_dependency git@gite.lirmm.fr:vrep-utils/vrep-api-wrapper
  build_git_dependency_no_test git@gite.lirmm.fr:multi-contact/mc_vrep

  cd $SOURCE_DIR
  if $WITH_HRP4
  then
    if [ ! -d vrep-hrp4/.git ]
    then
      git_clone git@gite.lirmm.fr:mc-hrp4/vrep_hrp.git vrep-hrp4
    else
      cd vrep-hrp4
      git_update
    fi
  fi

  cd $SOURCE_DIR
  if $WITH_HRP2
  then
    if [ ! -d vrep-hrp2/.git ]
    then
      git_clone git@gite.lirmm.fr:mc-hrp2/vrep-hrp2.git
    else
      cd vrep-hrp2
      git_update
    fi
  fi
fi

echo "Installation finished, please add the following lines to your .bashrc/.zshrc"
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
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "source $SOURCE_DIR/catkin_ws/devel/setup.bash"
  fi
fi
