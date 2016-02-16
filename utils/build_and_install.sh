#!/bin/sh -ex

##########################
#  --  Configuration --  #
##########################

readonly mc_rtc_dir=`cd $(dirname $0)/..; pwd`

readonly SOURCE_DIR=`cd $mc_rtc_dir/../; pwd`
readonly INSTALL_PREFIX="/tmp"
readonly WITH_ROS_SUPPORT="false"
VREP_PREFIX=

readonly BUILD_TYPE="RelWithDebInfo"
if command -v nproc
then
  BUILD_CORE=`nproc`
else
  BUILD_CORE=`sysctl -n hw.ncpu`
fi
readonly ROS_DISTRO=indigo
readonly ROS_APT_DEPENDENCIES="ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rviz-animated-view-controller"
ROS_GIT_DEPENDENCIES="git@gite.lirmm.fr:multi-contact/mc_ros#karim_drc git@gite.lirmm.fr:mc-hrp2/hrp2_drc#master git@gite.lirmm.fr:mc-hrp4/hrp4#master"
readonly git_clone="git clone --quiet --recursive"

SUDO_CMD=sudo
if [ -w $INSTALL_PREFIX ]
then
  SUDO_CMD=
fi

readonly gitlab_ci_yml=$mc_rtc_dir/.gitlab-ci.yml

export PATH=$INSTALL_PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$DYLD_LIBRARY_PATH
export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=$INSTALL_PREFIX/lib/python2.7/site-packages:$PYTHONPATH

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
  if ! command -v brew
  then
    /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  fi
  brew tap homebrew/science
  brew update
  brew install coreutils gnu-sed wget python cmake doxygen jsoncpp qhull tinyxml2 geos boost eigen || true
  # eigen3.pc is broken in brew release
  gsed -i -e's@Cflags: -Iinclude/eigen3@Cflags: -I/usr/local/include/eigen3@' /usr/local/lib/pkgconfig/eigen3.pc
else
  export OS=$(lsb_release -si)
  if [ $OS = Ubuntu ]
  then
    yaml_to_env "APT_DEPENDENCIES" $gitlab_ci_yml
    sudo apt-get update
    sudo apt-get install -qq cmake build-essential gfortran doxygen cython ${APT_DEPENDENCIES}
  else
    echo "This script does not support your OS: ${OS}, please contact the maintainer"
    exit 1
  fi
fi

###############################
#  --  Get Cython-0.23.4  --  #
###############################
cd $SOURCE_DIR
if [ ! -d Cython-0.23.4 ]
then
  wget --quiet http://cython.org/release/Cython-0.23.4.tar.gz
  tar xzf Cython-0.23.4.tar.gz
  cd Cython-0.23.4
  sudo python setup.py install
fi

#######################
#  --  Get Eigen  --  #
#######################
readonly EIGEN_VERSION=3.2.7
readonly EIGEN_HASH=b30b87236a1b

if [ ${OS} != Darwin ]
then
  if [ ! -d $SOURCE_DIR/eigen-eigen-${EIGEN_HASH} ]
  then
    # Checkout Eigen
    cd "$SOURCE_DIR"
    wget --quiet "http://bitbucket.org/eigen/eigen/get/${EIGEN_VERSION}.tar.gz"
    tar xzf ${EIGEN_VERSION}.tar.gz
    cd "$SOURCE_DIR/eigen-eigen-${EIGEN_HASH}/"
    mkdir -p build
    cd build
    # Build, make and install Eigen
    cmake .. -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
      -Dpkg_config_libdir:STRING="$INSTALL_PREFIX/lib" \
      -DEIGEN_INCLUDE_INSTALL_DIR:STRING="$INSTALL_PREFIX/include/eigen3" \
      ${CMAKE_ADDITIONAL_OPTIONS}
    make -j${BUILD_CORE}
    ${SUDO_CMD} make install
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

build_git_dependency()
{
  git_dependency_parsing $1
  echo "--> Compiling $git_dep (branch $git_dep_branch)"
  cd "$SOURCE_DIR"
  mkdir -p "$git_dep"
  if [ ! -d "$git_dep/.git" ]
  then
    $git_clone -b $git_dep_branch "$git_dep_uri" "$git_dep"
  fi
  mkdir -p $git_dep/build
  cd "$git_dep/build"
  cmake .. -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
           -DPYTHON_BINDING:BOOL=OFF \
           -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
           ${CMAKE_ADDITIONAL_OPTIONS}
  make -j${BUILD_CORE}
  ${SUDO_CMD} make install
}
##############################
#  --  GIT dependencies  --  #
##############################
yaml_to_env "GIT_DEPENDENCIES" $gitlab_ci_yml
if [ ${OS} = Darwin ]
then
  GIT_DEPENDENCIES=`echo $GIT_DEPENDENCIES|sed -e 's@jorisv/RBDyn@gergondet/RBDyn#topic/Win32@'`
fi
for package in ${GIT_DEPENDENCIES}; do
  build_git_dependency "$package"
done

################################
#  -- Handle ROS packages  --  #
################################
if $WITH_ROS_SUPPORT
then
  if [ ! -e /opt/ros/${ROS_DISTRO}/setup.sh ]
  then
    if [ $OS = Ubuntu ]
    then
      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -c -s` main" > /etc/apt/sources.list.d/ros-latest.list'
      wget http://packages.ros.org/ros.key -O - | ${SUDO_CMD} apt-key add -
      sudo apt-get update
      sudo apt-get install -qq ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-rosdoc-lite python-catkin-lint
    else
      echo "Please install ROS before continuing your installation or disable ROS support"
      exit 1
    fi
  fi
  . /opt/ros/${ROS_DISTRO}/setup.sh
  sudo apt-get install -qq ${ROS_APT_DEPENDENCIES}
  mkdir -p $SOURCE_DIR/catkin_ws/src
  cd $SOURCE_DIR/catkin_ws/src
  catkin_init_workspace
  for package in ${ROS_GIT_DEPENDENCIES}; do
    git_dependency_parsing $package
    cd $SOURCE_DIR
    if [ ! -d "$git_dep/.git" ]
    then
      $git_clone -b $git_dep_branch "$git_dep_uri" "$git_dep"
    fi
  done
  cd $SOURCE_DIR/catkin_ws
  catkin_make
  . $SOURCE_DIR/catkin_ws/devel/setup.sh
else
  ROS_GIT_DEPENDENCIES=`echo $ROS_GIT_DEPENDENCIES|sed -e's/hrp4#master/hrp4#noxacro/'`
  for package in ${ROS_GIT_DEPENDENCIES}; do
    git_dependency_parsing $package
    cd $SOURCE_DIR
    if [ ! -d "$git_dep/.git" ]
    then
      $git_clone -b $git_dep_branch "$git_dep_uri" "$git_dep"
    fi
  done
fi

##########################
#  --  Build mc_rtc  --  #
##########################
cd $mc_rtc_dir
mkdir -p build
cd build
if $WITH_ROS_SUPPORT
then
  cmake ../ -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
            -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
            ${CMAKE_ADDITIONAL_OPTIONS}
else
  cmake ../ -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
            -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX"\
            -DMC_ENV_DESCRIPTION_PATH:STRING="$SOURCE_DIR/mc_ros/mc_env_description"\
            -DHRP2_DRC_DESCRIPTION_PATH:STRING="$SOURCE_DIR/hrp2_drc/hrp2_drc_description"\
            -DHRP4_DESCRIPTION_PATH:STRING="$SOURCE_DIR/hrp4/hrp4_description" \
            ${CMAKE_ADDITIONAL_OPTIONS}
fi
make -j$BUILD_CORE
${SUDO_CMD} make install

####################################
#  --  Fetch sch python addon  --  #
####################################
cd $SOURCE_DIR
if [ ! -d sch-core-python/.git ]
then
  $git_clone https://github.com/jrl-umi3218/sch-core-python.git
fi
cd sch-core-python
mkdir -p ${INSTALL_PREFIX}/include/sch/Python
${SUDO_CMD} cp include/sch/Python/SCHAddon.h ${INSTALL_PREFIX}/include/sch/Python/

#############################
#  --  Build mc_cython  --  #
#############################
cd $SOURCE_DIR
if [ ! -d mc_cython/.git ]
then
  $git_clone git@gite.lirmm.fr:multi-contact/mc_cython
fi
cd mc_cython
if [ ! -e eigen/eigen.pyx ]
then
  python generate_pyx.py
fi
make -j$BUILD_CORE
# Make sure the python prefix exists
mkdir -p ${INSTALL_PREFIX}/lib/python`python -c "import sys;print '{0}.{1}'.format(sys.version_info.major, sys.version_info.minor)"`/site-packages
${SUDO_CMD} make install

#########################################
#  -- Setup VREP plugin and mc_vrep --  #
#########################################
if [ "x${VREP_PREFIX}" = "x" ]
then
  cd $SOURCE_DIR
  if [ $OS = Darwin ]
  then
    if [ ! -d V-REP_PRO_EDU_V3_2_3_rev4_Mac ]
    then
      wget http://coppeliarobotics.com/V-REP_PRO_EDU_V3_2_3_rev4_Mac.zip
      unzip V-REP_PRO_EDU_V3_2_3_rev4_Mac.zip
    fi
    VREP_PREFIX=$SOURCE_DIR/V-REP_PRO_EDU_V3_2_3_rev4_Mac
  else
    if [ ! -d V-REP_PRO_EDU_V3_2_3_rev4_64_Linux ]
    then
      wget http://coppeliarobotics.com/V-REP_PRO_EDU_V3_2_3_rev4_64_Linux.tar.gz
      tar xzf V-REP_PRO_EDU_V3_2_3_rev4_64_Linux.tar.gz
    fi
    VREP_PREFIX=$SOURCE_DIR/V-REP_PRO_EDU_V3_2_3_rev4_64_Linux
  fi
fi

cd $SOURCE_DIR
if [ ! -d mc_vrep/.git ]
then
  $git_clone git@gite.lirmm.fr:multi-contact/mc_vrep
fi
cp $SOURCE_DIR/mc_vrep/ext/extApiCustom.h $VREP_PREFIX/programming/include
cp $SOURCE_DIR/mc_vrep/src/vrep_remote_api/extApiCustomConst.h $VREP_PREFIX/programming/include
cp $SOURCE_DIR/mc_vrep/ext/simxCustomCmd.cpp $VREP_PREFIX/programming/v_repExtRemoteApi
cd $VREP_PREFIX/programming/v_repExtRemoteApi
make -j$BUILD_CORE
if [ $OS = Darwin ]
then
  cp lib/libv_repExtRemoteApi.dylib $VREP_PREFIX/vrep.app/Contents/MacOS/
else
  cp lib/libv_repExtRemoteApi.so $VREP_PREFIX
fi
cd $SOURCE_DIR/mc_vrep
cmake ../ -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
          -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
          ${CMAKE_ADDITIONAL_OPTIONS}
make
${SUDO_CMD} make install

cd $SOURCE_DIR
if [ ! -d vrep_hrp/.git ]
then
  $git_clone git@gite.lirmm.fr:mc-hrp4/vrep_hrp.git
fi

echo "Installation finished, please add the following lines to your .bashrc/.zshrc"
if [ ${OS} = Darwin ]
then
  echo """
  export PATH=$INSTALL_PREFIX/bin:\$PATH
  export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$DYLD_LIBRARY_PATH
  export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH
  export PYTHONPATH=$INSTALL_PREFIX/lib/python2.7/site-packages:\$PYTHONPATH
  """
else
  echo """
  export PATH=$INSTALL_PREFIX/bin:\$PATH
  export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$LD_LIBRARY_PATH
  export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH
  export PYTHONPATH=$INSTALL_PREFIX/lib/python2.7/site-packages:\$PYTHONPATH
  """
fi
