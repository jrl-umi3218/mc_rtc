#!/bin/bash

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

shopt -s expand_aliases

# display the list of parameters for autocompletion
if [[ $1 == --inputlist ]];
then
        echo -h --help -i --install-prefix -s --source-dir --with-ros-support --with-python-support \
              --python-user-install --python-force-python2 --python-force-python3 --python-build-2-and-3 \
              --with-lssol --with-hrp2 --with-hrp4 --with-hrp4j --with-hrp4cr --with-hrp5 --with-mc_udp --with-mc_openrtm \
              --build-type --build-testing --build-benchmarks --install-system-dependencies \
              --install-system-dependencies --clone-only --skip-update --skip-dirty-update --user-input \
              -j --build-core --ros-distro --allow-root
        exit
fi

##########################
#  --  Configuration --  #
##########################

readonly this_dir=`cd $(dirname $0); pwd`
readonly mc_rtc_dir=`cd $this_dir/..; pwd`
PYTHON_VERSION=`python -c 'import sys; print("{}.{}".format(sys.version_info.major, sys.version_info.minor))'`

. "$this_dir/build_and_install_default_config.sh"

echo_log()
{
  echo "$1" | $TEE -a $BUILD_LOGFILE
}

exec_log()
{
  echo "+ $* (pwd: `pwd`)"
  $* 2>&1 | $TEE -a $BUILD_LOGFILE
  return ${PIPESTATUS[0]}
}

exit_failure()
{
  echo_log "Installation failed."
  echo "Installation log has been written to $BUILD_LOGFILE"
  echo_log "Please fix the issues and re-run the script."
  exit 1
}

exit_if_error()
{
  if [ $? -ne 0 ]
  then
    echo_log "$1"
    exit_failure
  fi
}

mc_rtc_extra_steps()
{
  true
}

fix_ninja_perms()
{
  if [ -f .ninja_deps ]
  then
    owner="$(stat --format '%U' .ninja_deps)"
    if [ "x${owner}" != "x${USER}" ]
    then
      ${REQUIRED_SUDO} chown -R $USER .ninja_deps
    fi
  fi
  if [ -f .ninja_log ]
  then
    owner="$(stat --format '%U' .ninja_log)"
    if [ "x${owner}" != "x${USER}" ]
    then
      ${REQUIRED_SUDO} chown -R $USER .ninja_log
    fi
  fi
}

echo_log ""
echo_log "========================================"
echo_log "== mc_rtc build_and_install.sh script =="
echo_log "========================================"
echo_log ""


echo_log "-- Loaded default configuration from $this_dir/build_and_install_default_config.sh"
if [ -f "$this_dir/build_and_install_user_config.sh" ]; then
  . "$this_dir/build_and_install_user_config.sh"
  echo_log "-- Loaded user configuration from $this_dir/build_and_install_user_config.sh"
else
  echo_log "-- No user configuration file $this_dir/build_and_install_user_config.sh provided, using default configuration from $this_dir/build_and_install_default_config.sh"
  echo "   If you wish to create a custom user configuration:"
  echo "     - Copy the sample configuration: cp $this_dir/build_and_install_user_config.sample.sh $this_dir/build_and_install_user_config.sh"
  echo "     - Edit the options to your liking"
fi

case $BUILD_SUBDIR in
  /*) export BUILD_SUBDIR_IS_ABSOLUTE=true;;
  *) export BUILD_SUBDIR_IS_ABSOLUTE=false;;
esac

readonly HELP_STRING="$(basename $0) [OPTIONS] ...
    --help                (-h)               : print this help
    --install-prefix      (-i) PATH          : the directory used to install everything                (default $INSTALL_PREFIX)
    --source-dir          (-s) PATH          : the directory used to clone everything                  (default $SOURCE_DIR)
    --build-type               Type          : the build type to use                                   (default $BUILD_TYPE)
    --build-testing            {true, false} : whether to build and run unit tests                     (default $BUILD_TESTING)
    --build-benchmarks         {true, false} : whether to build and run benchmarks                     (default $BUILD_BENCHMARKS)
    --build-core          (-j) N             : number of cores used for building                       (default $BUILD_CORE)
    --with-lssol                             : enable LSSOL (requires multi-contact group access)      (default $WITH_LSSOL)
    --with-hrp2                              : enable HRP2 (requires mc-hrp2 group access)             (default $WITH_HRP2)
    --with-hrp4                              : enable HRP4 (requires mc-hrp4 group access)             (default $WITH_HRP4)
    --with-hrp4j                             : enable HRP4J (requires mc-hrp4 group access)            (default $WITH_HRP4J)
    --with-hrp4cr                            : enable HRP4CR (requires isri-aist group access)         (default $WITH_HRP4CR)
    --with-hrp5                              : enable HRP5 (requires mc-hrp5 group access)             (default $WITH_HRP5)
    --with-panda                             : enable Panda (requires ROS)                             (default $WITH_PANDA)
    --with-mc_openrtm                        : enable the mc_openrtm interface (requires hrpsys-base)  (default $WITH_MC_OPENRTM)
    --with-mc_udp                            : enable the mc_udp interface (requires hrpsys-base)      (default $WITH_MC_UDP)
    --with-python-support           {true, false} : whether to build with Python support               (default $WITH_PYTHON_SUPPORT)
    --python-user-install           {true, false} : whether to install Python bindings with user       (default $PYTHON_USER_INSTALL)
    --python-force-python2          {true, false} : whether to enforce the use of Python 2             (default $PYTHON_FORCE_PYTHON2)
    --python-force-python3          {true, false} : whether to enforce the use of Python 3             (default $PYTHON_FORCE_PYTHON3)
    --python-build-2-and-3          {true, false} : whether to build both Python 2 and Python 3        (default $PYTHON_BUILD_PYTHON2_AND_PYTHON3)
    --with-ros-support              {true, false} : whether to build with ROS support                  (default $WITH_ROS_SUPPORT)
    --ros-distro                    NAME          : the ros distro to use                              (default $ROS_DISTRO)
    --install-system-dependencies      {true, false} : whether to install system packages              (default $INSTALL_SYSTEM_DEPENDENCIES)
    --clone-only                       {true, false} : only perform cloning                            (default $CLONE_ONLY)
    --skip-update                      {true, false} : skip git update                                 (default $SKIP_UPDATE)
    --skip-dirty-update                {true, false} : skip git update if dirty repository             (default ${SKIP_DIRTY_UPDATE})
    --user-input                       {true, false} : ask the user confirmation                       (default ${ASK_USER_INPUT})
"
#helper for parsing
check_true_false()
{
    if [ "true" != "$2" ] && [ "false" != "$2" ]
    then
        echo "passed parameter '$2' as flag for '$1'. the parameter has to be 'true' or 'false'"
        exit_failure
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

        -s|--source-dir)
        i=$(($i+1))
        SOURCE_DIR="${!i}"
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

        --with-lssol)
          i=$(($i+1))
          WITH_LSSOL="${!i}"
          check_true_false --with-lssol "$WITH_LSSOL"
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

        --with-hrp4j)
        i=$(($i+1))
        WITH_HRP4J="${!i}"
        check_true_false --with-hrp4j "$WITH_HRP4J"
        ;;

        --with-hrp4cr)
        i=$(($i+1))
        WITH_HRP4CR="${!i}"
        check_true_false --with-hrp4cr "$WITH_HRP4CR"
        ;;

        --with-hrp5)
        i=$(($i+1))
        WITH_HRP5="${!i}"
        check_true_false --with-hrp5 "$WITH_HRP5"
        ;;

        --with-panda)
        i=$(($i+1))
        WITH_PANDA="${!i}"
        check_true_false --with-panda "$WITH_PANDA"
        ;;

        --with-mc_udp)
        i=$(($i+1))
        WITH_MC_UDP="${!i}"
        check_true_false --with-mc_udp "$WITH_MC_UDP"
        ;;

        --with-mc_openrtm)
        i=$(($i+1))
        WITH_MC_OPENRTM="${!i}"
        check_true_false --with-mc_openrtm "$WITH_MC_OPENRTM"
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

        --build-benchmarks)
        i=$(($i+1))
        BUILD_BENCHMARKS="${!i}"
        check_true_false --build-benchmarks "$BUILD_BENCHMARKS"
        ;;

        --install-system-dependencies)
        i=$(($i+1))
        INSTALL_SYSTEM_DEPENDENCIES="${!i}"
        check_true_false --install-system-dependencies "$INSTALL_SYSTEM_DEPENDENCIES"
        ;;

        --clone-only)
        i=$(($i+1))
        CLONE_ONLY="${!i}"
        check_true_false --clone-only "$CLONE_ONLY"
        ;;

        --skip-update)
        i=$(($i+1))
        SKIP_UPDATE="${!i}"
        check_true_false --skip-update "$SKIP_UPDATE"
        ;;

        --skip-dirty-update)
        i=$(($i+1))
        SKIP_DIRTY_UPDATE="${!i}"
        check_true_false --skip-dirty-update "$SKIP_DIRTY_UPDATE"
        ;;

        --user-input)
        i=$(($i+1))
        ASK_USER_INPUT="${!i}"
        check_true_false --user-input "$ASK_USER_INPUT"
        ;;

        -j|--build-core)
        i=$(($i+1))
        BUILD_CORE="${!i}"
        ;;

        --ros-distro)
        i=$(($i+1))
        ROS_DISTRO="${!i}"
        ;;

        --allow-root)
        i=$(($i+1))
        ALLOW_ROOT="${!i}"
        check_true_false --allow-root "$ALLOW_ROOT"
        ;;

        *)
        echo "unknown parameter $i ($key)"
        exit_failure
        ;;
    esac

    i=$(($i+1))
done

if [ "$ALLOW_ROOT" = "false" ] && [ $(id -u) -eq 0 ]
then
  echo_log "Please run this script as a non-root user. sudo permission will be asked where necessary."
  echo_log "You may force installation as root by setting ALLOW_ROOT=true (--allow-root true). Please note that this may have unintended consequences."
  exit_failure
fi

if [ $(id -u) -eq 0 ]
then
  export REQUIRED_SUDO=
else
  export REQUIRED_SUDO=sudo
fi

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
if $CLONE_ONLY
then
  readonly NOT_CLONE_ONLY=false
else
  readonly NOT_CLONE_ONLY=true
fi
export MC_LOG_UI_PYTHON_EXECUTABLE=python

alias git_clone="git clone --recursive"
git_update()
{
  git pull origin $1 && git submodule sync && git submodule update --init --recursive
}

install_apt()
{
  TO_INSTALL=
  for pkg in $*
  do
    has_package=`dpkg -l ${pkg} 2> /dev/null |grep "^ii" > /dev/null`
    if [ $? -ne 0 ]
    then
      TO_INSTALL="$TO_INSTALL $pkg"
    fi
  done
  if [ "${TO_INSTALL}" != "" ]
  then
    exec_log ${REQUIRED_SUDO} apt-get update
    exec_log ${REQUIRED_SUDO} apt-get -y install $*
  fi
  exit_if_error "-- [ERROR] Could not install one of the following packages ${TO_INSTALL}."
}

touch $BUILD_LOGFILE
exit_if_error "-- [ERROR] Could not create log file $BUILD_LOGFILE"
ln -sf $BUILD_LOGFILE "$LOG_PATH/build_and_install_warnings-latest.log"

echo_log "-- Log file will be written to $BUILD_LOGFILE "
echo_log ""
echo_log "-- Requested installation with the following options:"
echo_log "   INSTALL_PREFIX=$INSTALL_PREFIX"
echo_log "   SOURCE_DIR=$SOURCE_DIR"
echo_log "   WITH_ROS_SUPPORT=$WITH_ROS_SUPPORT"
echo_log "   WITH_PYTHON_SUPPORT=$WITH_PYTHON_SUPPORT"
echo_log "   PYTHON_FORCE_PYTHON2=$PYTHON_FORCE_PYTHON2"
echo_log "   PYTHON_FORCE_PYTHON3=$PYTHON_FORCE_PYTHON3"
echo_log "   PYTHON_BUILD_PYTHON2_AND_PYTHON3=$PYTHON_BUILD_PYTHON2_AND_PYTHON3"
echo_log "   BUILD_TYPE=$BUILD_TYPE"
echo_log "   INSTALL_SYSTEM_DEPENDENCIES=$INSTALL_SYSTEM_DEPENDENCIES"
echo_log "   BUILD_CORE=$BUILD_CORE"
echo_log "   BUILD_TESTING=$BUILD_TESTING"
echo_log "   BUILD_BENCHMARKS=$BUILD_BENCHMARKS"
echo_log "   CLONE_ONLY=$CLONE_ONLY"
echo_log "   WITH_LSSOL=$WITH_LSSOL"
echo_log "   WITH_HRP2=$WITH_HRP2"
echo_log "   WITH_HRP4=$WITH_HRP4"
echo_log "   WITH_HRP4J=$WITH_HRP4J"
echo_log "   WITH_HRP4CR=$WITH_HRP4CR"
echo_log "   WITH_HRP5=$WITH_HRP5"
echo_log "   WITH_PANDA=$WITH_PANDA"
echo_log "   WITH_MC_UDP=$WITH_MC_UDP"
echo_log "   MC_UDP_INSTALL_PREFIX=$MC_UDP_INSTALL_PREFIX"
echo_log "   WITH_MC_OPENRTM=$WITH_MC_OPENRTM"
echo_log "   MC_OPENRTM_INSTALL_PREFIX=$MC_OPENRTM_INSTALL_PREFIX"
echo_log "   SKIP_UPDATE=$SKIP_UPDATE"
echo_log "   SKIP_DIRTY_UPDATE=$SKIP_DIRTY_UPDATE"
echo_log "   BUILD_LOGFILE=$BUILD_LOGFILE"
echo_log "   ASK_USER_INPUT=$ASK_USER_INPUT"

if $WITH_PANDA
then
  if ! $WITH_ROS_SUPPORT
  then
    exit_failure "Panda robot cannot be installed without ROS support"
  fi
fi

##################################################
## Extra OS/Distribution specific configuration ##
##################################################
echo_log ""
echo_log "========================"
echo_log "== System information =="
echo_log "========================"
echo_log ""
if [[ $OSTYPE == "linux-gnu" ]]
then
  if ! command -v lsb_release &> /dev/null
  then
    echo_log "lsb_release must be installed for this script to work"
    exit_failure
  fi
  exec_log lsb_release -a
fi
exec_log cmake --version
exec_log python --version

echo_log "-- Loading extra configuration for $OSTYPE"
export SYSTEM_HAS_SPDLOG=OFF
export SYSTEM_HAS_NINJA=ON
export DISABLE_NINJA=OFF
if [[ $OSTYPE == "darwin"* ]]
then
  . $this_dir/config_build_and_install.macos.sh
elif [[ $OSTYPE == "linux-gnu" ]]
then
  if [ -f $this_dir/config_build_and_install.`lsb_release -sc`.sh ]
  then
    . $this_dir/config_build_and_install.`lsb_release -sc`.sh
    ROS_APT_DEPENDENCIES="ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-rosdoc-lite ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rviz"
    if $WITH_PANDA
    then
      ROS_APT_DEPENDENCIES="$ROS_APT_DEPENDENCIES ros-${ROS_DISTRO}-libfranka ros-${ROS_DISTRO}-franka-description"
    fi
  else
    ROS_DISTRO=""
    APT_DEPENDENCIES=""
    ROS_APT_DEPENDENCIES=""
  fi
else
  # Assume Windows
  . $this_dir/config_build_and_install.windows.sh
fi

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
if [ ! -d $SOURCE_DIR ]
then
  mkdir -p $SOURCE_DIR
fi


if [ "x$PYTHON_FORCE_PYTHON2" == xON ] ; then
  PYTHON_VERSION=`python2 -c 'import sys; print("{}.{}".format(sys.version_info.major, sys.version_info.minor))'`
elif [ "x$PYTHON_FORCE_PYTHON3" == xON ] ; then
  PYTHON_VERSION=`python3 -c 'import sys; print("{}.{}".format(sys.version_info.major, sys.version_info.minor))'`
fi
readonly PYTHON_VERSION

export PATH=$INSTALL_PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$DYLD_LIBRARY_PATH
export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages:$PYTHONPATH
echo_log "PATH: $PATH"
echo_log "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo_log "DYLD_LIBRARY_PATH: $DYLD_LIBRARY_PATH"
echo_log "PKG_CONFIG_PATH: $PKG_CONFIG_PATH"
echo_log "PYTHONPATH: $PYTHONPATH"

#make settings readonly
readonly INSTALL_PREFIX
readonly SOURCE_DIR
readonly WITH_PYTHON_SUPPORT
readonly WITH_PYTHON_SUPPORT
readonly PYTHON_FORCE_PYTHON2
readonly PYTHON_FORCE_PYTHON3
readonly PYTHON_BUILD_PYTHON2_AND_PYTHON3
readonly BUILD_TYPE
readonly INSTALL_SYSTEM_DEPENDENCIES
readonly BUILD_CORE
readonly BUILD_TESTING
readonly BUILD_BENCHMARKS
readonly CLONE_ONLY
readonly WITH_LSSOL
readonly WITH_HRP2
readonly WITH_HRP4
readonly WITH_HRP4J
readonly WITH_HRP4CR
readonly WITH_HRP5
readonly WITH_PANDA
readonly WITH_MC_OPENRTM
readonly MC_OPENRTM_INSTALL_PREFIX
readonly WITH_MC_UDP
readonly MC_UDP_INSTALL_PREFIX
readonly SKIP_UPDATE
readonly SKIP_DIRTY_UPDATE
readonly BUILD_LOGFILE
readonly ASK_USER_INPUT
readonly BUILD_SUBDIR

echo_log "-- Installing with the following options:"
echo_log "   INSTALL_PREFIX=$INSTALL_PREFIX"
echo_log "   SOURCE_DIR=$SOURCE_DIR"
echo_log "   WITH_ROS_SUPPORT=$WITH_ROS_SUPPORT"
echo_log "   WITH_PYTHON_SUPPORT=$WITH_PYTHON_SUPPORT"
echo_log "   PYTHON_FORCE_PYTHON2=$PYTHON_FORCE_PYTHON2"
echo_log "   PYTHON_FORCE_PYTHON3=$PYTHON_FORCE_PYTHON3"
echo_log "   PYTHON_BUILD_PYTHON2_AND_PYTHON3=$PYTHON_BUILD_PYTHON2_AND_PYTHON3"
echo_log "   BUILD_TYPE=$BUILD_TYPE"
echo_log "   INSTALL_SYSTEM_DEPENDENCIES=$INSTALL_SYSTEM_DEPENDENCIES"
echo_log "   BUILD_CORE=$BUILD_CORE"
echo_log "   BUILD_TESTING=$BUILD_TESTING"
echo_log "   BUILD_BENCHMARKS=$BUILD_BENCHMARKS"
echo_log "   CLONE_ONLY=$CLONE_ONLY"
echo_log "   WITH_LSSOL=$WITH_LSSOL"
echo_log "   WITH_HRP2=$WITH_HRP2"
echo_log "   WITH_HRP4=$WITH_HRP4"
echo_log "   WITH_HRP4J=$WITH_HRP4J"
echo_log "   WITH_HRP4CR=$WITH_HRP4CR"
echo_log "   WITH_HRP5=$WITH_HRP5"
echo_log "   WITH_PANDA=$WITH_PANDA"
echo_log "   WITH_MC_UDP=$WITH_MC_UDP"
echo_log "   MC_UDP_INSTALL_PREFIX=$MC_UDP_INSTALL_PREFIX"
echo_log "   WITH_MC_OPENRTM=$WITH_MC_OPENRTM"
echo_log "   MC_OPENRTM_INSTALL_PREFIX=$MC_OPENRTM_INSTALL_PREFIX"
echo_log "   SKIP_UPDATE=$SKIP_UPDATE"
echo_log "   SKIP_DIRTY_UPDATE=$SKIP_DIRTY_UPDATE"
echo_log "   BUILD_LOGFILE=$BUILD_LOGFILE"
echo_log "   ASK_USER_INPUT=$ASK_USER_INPUT"
echo_log "   ROS_DISTRO=$ROS_DISTRO"
echo_log "   APT_DEPENDENCIES=$APT_DEPENDENCIES"
echo_log "   ROS_APT_DEPENDENCIES=$ROS_APT_DEPENDENCIES"
echo_log "   SUDO_CMD=$SUDO_CMD"

###################################
#  --  APT/Brew dependencies  --  #
###################################
echo_log ""
echo_log "===================================="
echo_log "== Installing system dependencies =="
echo_log "===================================="
echo_log
if [[ $OSTYPE == "darwin"* ]]
then
  export OS=macOS
  # Install brew on the system
  if $INSTALL_SYSTEM_DEPENDENCIES && $NOT_CLONE_ONLY
  then
    if ! command -v brew
    then
      /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    fi
    brew update
    brew install $BREW_DEPENDENCIES
    brew upgrade $BREW_DEPENDENCIES
    if [ "x$WITH_PYTHON_SUPPORT" == xON ] && $NOT_CLONE_ONLY
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
    fi
    mc_rtc_extra_steps
  else
    echo_log "-- Skip installation of system dependencies"
  fi
elif [[ $OSTYPE == "linux-gnu" ]]
then
  export OS=$(lsb_release -si)
  if [ $OS = Ubuntu -o $OS = Debian ]
  then
    if $INSTALL_SYSTEM_DEPENDENCIES && $NOT_CLONE_ONLY
    then
      install_apt ${APT_DEPENDENCIES}
      mc_rtc_extra_steps
    else
      echo_log "-- Skip installation of system dependencies"
    fi
  else
    echo_log "-- [WARNING] This script does not support your OS: ${OS}, assuming you have installed the required system dependencies already"
  fi
else
  export OS=Windows
  if [ "x$WITH_PYTHON_SUPPORT" == xON ] && $NOT_CLONE_ONLY
  then
    pip install --user ${PIP_DEPENDENCIES}
  fi
  mc_rtc_extra_steps
fi

echo_log ""
echo_log "-- [SUCCESS] Successfully installed system dependencies"
echo_log ""

###############################################
#  -- Check python/pip coherency if needed -- #
###############################################

if [ "x$WITH_PYTHON_SUPPORT" == xON ] && [ "x$PYTHON_FORCE_PYTHON2" == xOFF ] && [ "x$PYTHON_FORCE_PYTHON3" == xOFF ]
then
  if ! pip --version | grep -q "`python -c 'import sys; print(\"python {}.{}\".format(sys.version_info.major, sys.version_info.minor));'`"
  then
    echo_log "The pip command does not match the corresponding python version, this will lead to errors"
    echo_log "Either fix your system or use --python-force-python2 true or --python-force-python3 true"
  fi
fi

if [ "x$WITH_PYTHON_SUPPORT" == xON ] && ( [ "x$PYTHON_FORCE_PYTHON2" == xON ] || [ "x$PYTHON_BUILD_PYTHON2_AND_PYTHON3" == xON ] )
then
  if ! pip2 --version | grep -q "`python2 -c 'import sys; print(\"python {}.{}\".format(sys.version_info.major, sys.version_info.minor));'`"
  then
    echo_log "The pip2 command does not match the corresponding python2 version, this will lead to errors"
    echo_log "Resolve the issue at your system level"
  fi
fi

if [ "x$WITH_PYTHON_SUPPORT" == xON ] && ( [ "x$PYTHON_FORCE_PYTHON3" == xON ] || [ "x$PYTHON_BUILD_PYTHON3_AND_PYTHON3" == xON ] )
then
  if ! pip3 --version | grep -q "`python3 -c 'import sys; print(\"python {}.{}\".format(sys.version_info.major, sys.version_info.minor));'`"
  then
    echo_log "The pip3 command does not match the corresponding python3 version, this will lead to errors"
    echo_log "Resolve the issue at your system level"
  fi
fi

########################
##  -- Install ROS --  #
########################

if $WITH_ROS_SUPPORT
then
  echo_log "================================"
  echo_log "== Setting up ROS environment =="
  echo_log "================================"
  if [ ! -e /opt/ros/${ROS_DISTRO}/setup.bash ] && $NOT_CLONE_ONLY
  then
    if [ $OS = Ubuntu -o $OS = Debian ]
    then
      ${REQUIRED_SUDO} mkdir -p /etc/apt/sources.list.d/
      ${REQUIRED_SUDO} sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -c -s` main" > /etc/apt/sources.list.d/ros-latest.list'
      wget http://packages.ros.org/ros.key -O - | ${REQUIRED_SUDO} apt-key add -
    else
      echo_log "Please install ROS and the required dependencies (${ROS_APT_DEPENDENCIES}) before continuing your installation or disable ROS support"
      exit_failure
    fi
  fi
  if [ $OS = Ubuntu -o $OS = Debian ]
  then
    install_apt $ROS_APT_DEPENDENCIES
  fi
  if $NOT_CLONE_ONLY
  then
    . /opt/ros/${ROS_DISTRO}/setup.bash
  fi
  CATKIN_DATA_WORKSPACE=$SOURCE_DIR/catkin_data_ws
  CATKIN_DATA_WORKSPACE_SRC=${CATKIN_DATA_WORKSPACE}/src
  if [[ ! -f $CATKIN_DATA_WORKSPACE/devel/setup.bash ]]
  then
    mkdir -p ${CATKIN_DATA_WORKSPACE_SRC}
    if $NOT_CLONE_ONLY
    then
      cd ${CATKIN_DATA_WORKSPACE_SRC}
      catkin_init_workspace || true
      cd ${CATKIN_DATA_WORKSPACE}
      catkin_make
      . $CATKIN_DATA_WORKSPACE/devel/setup.bash
    fi
  else
    . $CATKIN_DATA_WORKSPACE/devel/setup.bash
  fi
  CATKIN_WORKSPACE=$SOURCE_DIR/catkin_ws
  CATKIN_WORKSPACE_SRC=${CATKIN_WORKSPACE}/src
  if [[ ! -f $CATKIN_WORKSPACE/devel/setup.bash ]]
  then
    mkdir -p ${CATKIN_WORKSPACE_SRC}
    if $NOT_CLONE_ONLY
    then
      cd ${CATKIN_WORKSPACE_SRC}
      catkin_init_workspace || true
      cd ${CATKIN_WORKSPACE}
      catkin_make
      . $CATKIN_WORKSPACE/devel/setup.bash
    fi
  else
    . $CATKIN_WORKSPACE/devel/setup.bash
  fi
fi

echo_log ""
echo_log "-- [SUCCESS] ROS environment setup completed"
echo_log ""

#########################################
## -- Check local git repositoriees -- ##
#########################################
echo_log "==================================================="
echo_log "== Checking/updating/cloning local repositories  =="
echo_log "==================================================="
echo_log ""
echo "The script will ensure that you have clean repositories, i.e.:"
echo "- You don't have unstaged changes"
echo "- Your index doesn't contain uncommited changes"
echo "- You are not on an unexpected branch"
echo ""

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
  cd "$2"
  mkdir -p "$git_dep"
  if [ ! -d "$git_dep/.git" ]
  then
    # Doing git clone -b tag uri dep directly results in detached HEAD state
    # Hence doing it in two steps instead
    if ! git_clone "$git_dep_uri" "$git_dep"; then
      echo_log "[ERROR] Failed to clone ${git_dep_uri}"
      exit_failure
    fi
    cd "$2/$git_dep"
    if ! git checkout "$git_dep_branch" -B $git_dep_branch; then
      if ! git checkout "origin/$git_dep_branch" -B $git_dep_branch; then
        echo_log "[ERROR] Failed to checkout branch ${git_dep_branch}"
      fi
    fi
    git submodule sync --recursive && git submodule update --init --recursive
  else
    if $SKIP_UPDATE
    then
      return
    fi
    pushd . > /dev/null
    cd "$2/$git_dep"
    git_update $git_dep_branch
    exit_if_error "Git Update failed for ${git_dep}"
    popd > /dev/null
  fi
}

check_clean_work_tree ()
{
  err=0
  # Update the index
  git update-index -q --ignore-submodules --refresh

  # Disallow unstaged changes in the working tree
  if ! git diff-files --quiet --ignore-submodules --
  then
      echo_log "-- [ERROR] You have unstaged changes. Please commit or stash them."
      git diff-files --name-status -r --ignore-submodules -- >&2
      err=1
  fi

  # Disallow uncommitted changes in the index
  if ! git diff-index --cached --quiet HEAD --ignore-submodules --
  then
      echo_log "-- [ERROR] Your index contains uncommitted changes. Please commit or stash them"
      git diff-index --cached --name-status -r --ignore-submodules HEAD -- >&2
      err=1
  fi

  branch_name="`git rev-parse --abbrev-ref HEAD`"
  if [[ $git_dep_branch != $branch_name ]] && [[ "heads/$git_dep_branch" != $branch_name ]]; then
    if [ $err == 0 ]
    then
      echo_log "-- [WARNING] You were previously on branch $branch_name but the required branch is $git_dep_branch"
      echo_log "             Local repository is clean with no uncommited or unstashed changes, switching to branch $git_deb_branch"
      git checkout $git_dep_branch -B $git_dep_branch
      exit_if_error "-- [ERROR] Failed to change to required branch $git_dep_branch of repository $git_dep"
      err=0
    else
      echo_log "-- [ERROR] Expected branch $git_dep_branch but you are currently on $branch_name. Please commit or stash your local changes and switch to branch $git_dep_branch"
      err=1
    fi
  fi
  return $err
}

# Checks if the repository has already been cloned, in which case determine if
# the local state is clean (no uncommited changes, correct branch).
# If necessary, attempts to update the repository
check_and_clone_git_dependency()
{
  repo=$1
  git_dependency_parsing $repo
  source_dir="$2"
  repo_dir="$2/$git_dep"
  echo
  echo "-- Fetching changes in repository $git_dep (branch ${git_dep_branch}, uri: ${git_dep_uri})"
  if [[ -d $repo_dir ]]; then
    cd $repo_dir
    if [[ ! -d ".git" ]]; then
      echo_log "-- [ERROR]: local folder ${repo_dir} exists but is not a git repository. Please delete it an retry the script."
      exit_failure
    fi
    echo_log "-- [OK] Found local repository for ${git_dep} in ${repo_dir}"

    if check_clean_work_tree; then
      # Ensure that the remote is correct
      echo "-- Checking remote URL"
      remote="`git remote get-url origin`"
      if  [[ "$remote" == *"github"* ]] && [[ $git_dep_uri != *"github"* ]] ||  [[ "$remote" == *"gite"* ]] && [[ $git_dep_uri != *"gite"* ]]
      then
        echo_log "-- The remote has changed for $repo, previous remote: $remote, desired remote $git_dep_uri."

        if $ASK_USER_INPUT
        then
          read -r -p "--> Would you like to update to the new remote (warning this will overwrite the local master branch)? [y/N] " response
          if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]
          then
            echo_log "-- Updating the remote url to $git_dep_uri"
            git remote set-url origin $git_dep_uri
            git remote update origin
            exit_if_error "Failed to update the remote uri to $git_dep_uri"
            git fetch origin
            exit_if_error "Failed to fetch the remote uri to $git_dep_uri"
            git reset --hard origin/master
            exit_if_error "Failed to update to the the remote's master branch (new remote: $git_dep_uri)"
          else
            echo_log "-- Installation stopped because the locate remote $remote does not match the local remote uri ."
            echo_log "   Please manually update the remote to $git_dep_uri and rerun the script."
            exit_failure
          fi
        fi
      fi
      prev_commit="`git rev-parse HEAD`"
      echo_log "-- [OK] repository is clean"
      echo_log "-- Attempting to update local repository from remote branch ${git_dep_branch}..."
      clone_git_dependency $repo $source_dir
      cd $repo_dir
      curr_commit="`git rev-parse HEAD`"
      if [[ "$prev_commit" == "$curr_commit" ]]; then
        echo_log "-- [OK] Repository ${git_dep} was already on the latest commit $prev_commit (no update)"
      else
        echo_log "-- [OK] Repository ${git_dep} updated from commit $prev_commit to $curr_commit"
      fi
    else
      if $SKIP_DIRTY_UPDATE; then
        echo_log "-- [WARNING] Repository ${git_dep} is dirty (local changes/wrong branch), but IGNORE_DIRTY_REPOSITORIES=true: skipping update and attempting to continue without changes"
      else
        echo_log "-- [ERROR] Your local repository for $git_dep is dirty"
        echo_log "   Please ensure that you have no local uncommited changes and that you are on the expected branch ($git_dep_branch)"
        exit_failure
      fi
    fi
  else
    echo_log "-- Cloning ${git_dep} from ${git_dep_uri}"
    clone_git_dependency $repo $source_dir
    echo_log "-- [OK] Successfully cloned repository ${repo} with branch ${git_dep_branch} from ${git_dep_uri} to ${repo_dir}"
  fi
  echo
}

# If the dependencies have already been cloned, check if the local state of the repository is clean before upgrading
GIT_DEPENDENCIES="loco-3d/ndcurves#v1.1.2 jrl-umi3218/SpaceVecAlg jrl-umi3218/state-observation jrl-umi3218/sch-core jrl-umi3218/RBDyn jrl-umi3218/eigen-qld jrl-umi3218/eigen-quadprog jrl-umi3218/Tasks"
if [ "x$SYSTEM_HAS_SPDLOG" == xOFF ]
then
  GIT_DEPENDENCIES="gabime/spdlog#v1.6.1 $GIT_DEPENDENCIES"
fi

for repo in $GIT_DEPENDENCIES; do
  check_and_clone_git_dependency $repo $SOURCE_DIR
done

if [ "x$WITH_PYTHON_SUPPORT" == xON ]
then
  check_and_clone_git_dependency jrl-umi3218/Eigen3ToPython $SOURCE_DIR
  check_and_clone_git_dependency jrl-umi3218/sch-core-python $SOURCE_DIR
fi

if $WITH_ROS_SUPPORT
then
  check_and_clone_git_dependency jrl-umi3218/mc_rtc_data $CATKIN_DATA_WORKSPACE_SRC
  check_and_clone_git_dependency jrl-umi3218/mc_rtc_msgs $CATKIN_DATA_WORKSPACE_SRC
  check_and_clone_git_dependency jrl-umi3218/mc_rtc_ros $CATKIN_WORKSPACE_SRC
else
  check_and_clone_git_dependency jrl-umi3218/mc_rtc_data $SOURCE_DIR
fi

echo_log "-- [OK] All manadatory repositories successfuly cloned or updated"

################################
#  --  Fetch extra modules  -- #
################################
if $WITH_LSSOL
then
  check_and_clone_git_dependency git@gite.lirmm.fr:multi-contact/eigen-lssol $SOURCE_DIR
fi

if $WITH_HRP2
then
  if $WITH_ROS_SUPPORT
  then
    check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp2/hrp2_drc_description#main $CATKIN_DATA_WORKSPACE_SRC
    echo_log "-- [OK] Successfully cloned and updated the robot description to $git_dep to $repo_dir (catkin)"
  else
    check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp2/hrp2_drc_description#main $SOURCE_DIR
    echo_log "-- [OK] Successfully cloned and updated the robot description $git_dep to $repo_dir (no catkin)"
  fi
  check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp2/mc-hrp2 $SOURCE_DIR
  echo_log "-- [OK] Successfully cloned and updated the robot module $git_dep to $repo_dir"
fi

if $WITH_HRP4
then
  if $WITH_ROS_SUPPORT
  then
    check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp4/hrp4 $CATKIN_DATA_WORKSPACE_SRC
    echo_log "-- [OK] Successfully cloned and updated the robot description to $git_dep to $repo_dir (catkin)"
  else
    check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp4/hrp4 $SOURCE_DIR
    echo_log "-- [OK] Successfully cloned and updated the robot description $git_dep to $repo_dir (no catkin)"
  fi
  check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp4/mc-hrp4 $SOURCE_DIR
  echo_log "-- [OK] Successfully cloned and updated the robot module $git_dep to $repo_dir"
fi

if $WITH_HRP4J
then
  if $WITH_ROS_SUPPORT
  then
    check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp4/hrp4j_description $CATKIN_DATA_WORKSPACE_SRC
    echo_log "-- [OK] Successfully cloned and updated the robot description to $git_dep to $repo_dir (catkin)"
  else
    check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp4/hrp4j_description $SOURCE_DIR
    echo_log "-- [OK] Successfully cloned and updated the robot description $git_dep to $repo_dir (no catkin)"
  fi
  check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp4/mc_hrp4j $SOURCE_DIR
  echo_log "-- [OK] Successfully cloned and updated the robot module $git_dep to $repo_dir"
fi

if $WITH_HRP4CR
then
  if $WITH_ROS_SUPPORT
  then
    check_and_clone_git_dependency 'isri-aist/hrp4cr_description#main' $CATKIN_DATA_WORKSPACE_SRC
    echo_log "-- [OK] Successfully cloned and updated the robot description to $git_dep to $repo_dir (catkin)"
  else
    check_and_clone_git_dependency 'isri-aist/hrp4cr_description#main' $SOURCE_DIR
    echo_log "-- [OK] Successfully cloned and updated the robot description $git_dep to $repo_dir (no catkin)"
  fi
  check_and_clone_git_dependency 'isri-aist/mc_hrp4cr#main'  $SOURCE_DIR
  echo_log "-- [OK] Successfully cloned and updated the robot module $git_dep to $repo_dir"
fi

if $WITH_HRP5
then
  if $WITH_ROS_SUPPORT
  then
    check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp5/hrp5_p_description $CATKIN_DATA_WORKSPACE_SRC
    echo_log "-- [OK] Successfully cloned and updated the robot description $git_dep to $repo_dir (catkin)"
  else
    check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp5/hrp5_p_description $SOURCE_DIR
    echo_log "-- [OK] Successfully cloned and updated the robot description $git_dep to $repo_dir (no catkin)"
  fi
  check_and_clone_git_dependency git@gite.lirmm.fr:mc-hrp5/mc_hrp5_p $SOURCE_DIR
  echo_log "-- [OK] Successfully cloned and updated the robot module $git_dep to $repo_dir"
fi

if $WITH_PANDA
then
  check_and_clone_git_dependency jrl-umi3218/mc_panda $SOURCE_DIR
  echo_log "-- [OK] Successfully cloned and updated the robot module $git_dep to $repo_dir"
fi

if $WITH_MC_UDP
then
  check_and_clone_git_dependency jrl-umi3218/mc_udp $SOURCE_DIR
  echo_log "-- [OK] Successfully cloned and updated the interface $git_dep to $repo_dir"
fi

if $WITH_MC_OPENRTM
then
  check_and_clone_git_dependency jrl-umi3218/mc_openrtm $SOURCE_DIR
  echo_log "-- [OK] Successfully cloned and updated the interface $git_dep to $repo_dir"
fi

echo_log "-- [OK] All extra repositiories have been successfully cloned or updated"
echo_log "-- [SUCCESS] All repositories have been successfully cloned or updated"

if $CLONE_ONLY
then
  echo_log "-- [INFO] The script was executed with CLONE_ONLY=true, stopping now."
  echo_log "   Use CLONE_ONLY=false if you wish to build and install."
  exit 0
fi

echo_log ""
echo_log "==================================="
echo_log "== Building all git dependencies =="
echo_log "==================================="
echo_log ""

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
  # Build and install new version
  exec_log cmake --build . --config ${BUILD_TYPE}
  exit_if_error "[ERROR] Build failed for $1"
  # Uninstall previously installed files
  if [ -f install_manifest.txt ]
  then
    exec_log ${SUDO_CMD} cmake --build . --target uninstall --config ${BUILD_TYPE}
    if [ $? -ne 0 ]
    then
      echo_log "-- [WARNING] Uninstallation failed for $1"
    fi
  fi
  exec_log ${SUDO_CMD} cmake --build . --target install --config ${BUILD_TYPE}
  exit_if_error "-- [ERROR] Installation failed for $1"
  fix_ninja_perms
}

test_project()
{
  exec_log ctest -V -C ${BUILD_TYPE}
  if [ $? -ne 0 ]
  then
    if [ ! -z $2 ]
    then
      echo_log "$1 testing failed, assuming you need to rebuild your Python bindings"
      if [ "x$PYTHON_BUILD_PYTHON2_AND_PYTHON3" == xON ]
      then
        exec_log cmake --build . --config ${BUILD_TYPE} --target force-$2-python2-bindings
        exec_log cmake --build . --config ${BUILD_TYPE} --target force-$2-python3-bindings
      elif [ "x$PYTHON_FORCE_PYTHON2" == xON ]
      then
        exec_log cmake --build . --config ${BUILD_TYPE} --target force-$2-python2-bindings
      elif [ "x$PYTHON_FORCE_PYTHON3" == xON ]
      then
        exec_log cmake --build . --config ${BUILD_TYPE} --target force-$2-python3-bindings
      else
        exec_log cmake --build . --config ${BUILD_TYPE} --target force-$2-python-bindings
      fi
      exec_log cmake --build . --config ${BUILD_TYPE}
      exec_log ${SUDO_CMD} cmake --build . --target install --config ${BUILD_TYPE}
      fix_ninja_perms
      exit_if_error "[ERROR] Build failed for $1"
      exit_if_error "-- [ERROR] Installation failed for $1"
      exec_log ctest -V -C ${BUILD_TYPE}
      exit_if_error "-- [ERROR] Testing is still failing for $1, please investigate deeper"
    else
      echo_log "-- [ERROR] Testing failed for $1"
      exit_failure
    fi
  fi
}

build_git_dependency_configure_and_build()
{
  git_dependency_parsing $1
  echo "--> Compiling $git_dep (branch $git_dep_branch)"
  if $BUILD_SUBDIR_IS_ABSOLUTE
  then
    mkdir -p "$BUILD_SUBDIR/$git_dep"
    cd "$BUILD_SUBDIR/$git_dep"
  else
    mkdir -p "$SOURCE_DIR/$git_dep/$BUILD_SUBDIR"
    # Add the build subdirecory to the ignored files list if it is not ignored already
    if ! grep -Fxq "/$BUILD_SUBDIR/" $SOURCE_DIR/$git_dep/.git/info/exclude ;
    then
      echo "/$BUILD_SUBDIR/" >> $SOURCE_DIR/$git_dep/.git/info/exclude ;
    fi
    cd "$SOURCE_DIR/$git_dep/$BUILD_SUBDIR"
  fi
  if [[ $OS == "Windows" ]]
  then
    hide_sh
  fi
  custom_install_prefix=$INSTALL_PREFIX
  if [ ! -z "$2" ]
  then
    custom_install_prefix="$2"
  fi
  cmake_generator=""
  if [ "x$SYSTEM_HAS_NINJA" == xON ] && [ "x$DISABLE_NINJA" != xON ] && [ ! -f Makefile ]
  then
    cmake_generator="-GNinja"
  fi
  fix_ninja_perms
  exec_log cmake $SOURCE_DIR/$git_dep -DCMAKE_INSTALL_PREFIX:STRING="$custom_install_prefix" \
                  -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON \
                  -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
                  -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
                  -DPYTHON_BINDING_FORCE_PYTHON2:BOOL=${PYTHON_FORCE_PYTHON2} \
                  -DPYTHON_BINDING_FORCE_PYTHON3:BOOL=${PYTHON_FORCE_PYTHON3} \
                  -DPYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON3:BOOL=${PYTHON_BUILD_PYTHON2_AND_PYTHON3} \
                  -DMC_LOG_UI_PYTHON_EXECUTABLE:STRING="${MC_LOG_UI_PYTHON_EXECUTABLE}" \
                  -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
                  ${cmake_generator} \
                  ${CMAKE_ADDITIONAL_OPTIONS}
  exit_if_error "-- [ERROR] CMake configuration failed for $git_dep"
  build_project $git_dep
  if [[ $OS == "Windows" ]]
  then
    restore_path
  fi
}

build_git_dependency()
{
  echo_log "-- Building git dependency $1 (with test)"
  if $BUILD_TESTING
  then
    build_git_dependency_configure_and_build $1
    test_project $1 $2
  else
    build_git_dependency_no_test $1
  fi
}

build_git_dependency_no_test()
{
  echo_log "-- Building git dependency $1 (no test)"
  OLD_CMAKE_OPTIONS="${CMAKE_ADDITIONAL_OPTIONS}"
  export CMAKE_ADDITIONAL_OPTIONS="${OLD_CMAKE_OPTIONS} -DBUILD_TESTING:BOOL=OFF"
  build_git_dependency_configure_and_build $1 $2
  export CMAKE_ADDITIONAL_OPTIONS="${OLD_CMAKE_OPTIONS}"
}

build_catkin_workspace()
{
  if $CLONE_ONLY
  then
    return
  fi
  echo_log "-- Building catkin workspace $1"
  cd $1
  exec_log catkin_make
  exit_if_error "catkin_build failed for $git_dep"
  echo_log "-- [OK] Successfully built $1"
}



###############################
##  --  GIT dependencies  --  #
###############################

export OLD_CMAKE_OPTIONS="${CMAKE_ADDITIONAL_OPTIONS}"
if [ "x$SYSTEM_HAS_SPDLOG" == xOFF ]
then
  export CMAKE_ADDITIONAL_OPTIONS="-DSPDLOG_BUILD_EXAMPLE:BOOL=OFF -DSPDLOG_BUILD_SHARED:BOOL=ON ${CMAKE_ADDITIONAL_OPTIONS}"
  build_git_dependency_no_test gabime/spdlog
fi
export CMAKE_ADDITIONAL_OPTIONS="-DBUILD_PYTHON_INTERFACE:BOOL=OFF ${OLD_CMAKE_OPTIONS}"
build_git_dependency_no_test loco-3d/ndcurves
build_git_dependency jrl-umi3218/state-observation
export CMAKE_ADDITIONAL_OPTIONS="${OLD_CMAKE_OPTIONS}"
if [ "x$WITH_PYTHON_SUPPORT" == xON ]
then
  build_git_dependency jrl-umi3218/Eigen3ToPython eigen
fi
build_git_dependency jrl-umi3218/SpaceVecAlg sva
export CMAKE_ADDITIONAL_OPTIONS="-DCMAKE_CXX_STANDARD=11 ${OLD_CMAKE_OPTIONS}"
build_git_dependency jrl-umi3218/sch-core
if [ "x$WITH_PYTHON_SUPPORT" == xON ]
then
  build_git_dependency jrl-umi3218/sch-core-python sch
fi
export CMAKE_ADDITIONAL_OPTIONS="${OLD_CMAKE_OPTIONS}"
build_git_dependency jrl-umi3218/RBDyn rbdyn
export DISABLE_NINJA=ON
build_git_dependency jrl-umi3218/eigen-qld eigen_qld
build_git_dependency jrl-umi3218/eigen-quadprog
if $WITH_LSSOL
then
  echo_log "-- Building with eigen-lssol support (WITH_LSSOL=true)"
  build_git_dependency git@gite.lirmm.fr:multi-contact/eigen-lssol
  echo_log "-- [OK] Successfully built $git_dep to $repo_dir"
fi
export DISABLE_NINJA=OFF

build_git_dependency jrl-umi3218/Tasks tasks

if $WITH_ROS_SUPPORT
then
  build_catkin_workspace $CATKIN_DATA_WORKSPACE
else
  build_git_dependency jrl-umi3218/mc_rtc_data
fi

echo_log "-- [SUCCESS] All mandatory dependencies have been successfully built, tested and installed"

##########################
#  --  Build mc_rtc  --  #
##########################
echo_log ""
echo_log "====================="
echo_log "== Building mc_rtc =="
echo_log "====================="
echo_log ""

cd $mc_rtc_dir
git remote update origin
current_commit=`git rev-parse HEAD`
current_branch_name="`git rev-parse --abbrev-ref HEAD`"
remote_commit=`git rev-parse master@{upstream}`
if [[ "$current_commit" != "$remote_commit"  ]]
then
  echo "-- [WARNING] Would compile mc_rtc from commit $current_commit (currently on branch ${current_branch_name}) but the remote master branch is at $remote_commit"
  if $ASK_USER_INPUT
  then
    read -r -p "Are you sure? [y/N] " response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]
    then
      echo_log "-- Building mc_rtc from commit $current_commit"
    else
      echo_log "-- Installation manually cancelled because mc_rtc would have been built from commit $current_commit but the remote master branch is at $remote_commit"
      echo_log "   Please make sure mc_rtc is up-to date with the remote master branch and try again."
      exit_failure
    fi
  fi
fi
if ! $SKIP_UPDATE
then
  git submodule sync || true
  git submodule update --init
  exit_if_error "-- [ERROR] Failed to update submodules"
fi
if $BUILD_SUBDIR_IS_ABSOLUTE
then
  mkdir -p "$BUILD_SUBDIR/mc_rtc/"
  cd "$BUILD_SUBDIR/mc_rtc"
else
  mkdir -p $BUILD_SUBDIR
  # Add the build subdirecory to the ignored files list if it is not ignored already
  if ! grep -Fxq "/$BUILD_SUBDIR/" .git/info/exclude ;
  then
    echo "/$BUILD_SUBDIR/" >> .git/info/exclude ;
  fi
  cd $BUILD_SUBDIR
fi
if $BUILD_TESTING
then
  BUILD_TESTING_OPTION=ON
else
  BUILD_TESTING_OPTION=OFF
fi
if $BUILD_BENCHMARKS
then
  BUILD_BENCHMARKS_OPTION=ON
else
  BUILD_BENCHMARKS_OPTION=OFF
fi
if ! $WITH_ROS_SUPPORT
then
  CMAKE_ADDITIONAL_OPTIONS="${CMAKE_ADDITIONAL_OPTIONS} -DDISABLE_ROS=ON"
else
  CMAKE_ADDITIONAL_OPTIONS="${CMAKE_ADDITIONAL_OPTIONS} -DDISABLE_ROS=OFF"
fi
cmake_generator=""
if [ "x$SYSTEM_HAS_NINJA" == xON ] && [ "x$DISABLE_NINJA" != xON ] && [ ! -f Makefile ]
then
  cmake_generator="-GNinja"
fi
fix_ninja_perms
exec_log cmake $mc_rtc_dir -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
                   -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON \
                   -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
                   -DBUILD_TESTING:BOOL=${BUILD_TESTING_OPTION} \
                   -DBUILD_BENCHMARKS:BOOL=${BUILD_BENCHMARKS_OPTION} \
                   -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
                   -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
                   -DPYTHON_BINDING_FORCE_PYTHON2:BOOL=${PYTHON_FORCE_PYTHON2} \
                   -DPYTHON_BINDING_FORCE_PYTHON3:BOOL=${PYTHON_FORCE_PYTHON3} \
                   -DPYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON3:BOOL=${PYTHON_BUILD_PYTHON2_AND_PYTHON3} \
                   -DMC_LOG_UI_PYTHON_EXECUTABLE:STRING="${MC_LOG_UI_PYTHON_EXECUTABLE}" \
                   ${cmake_generator} \
                   ${CMAKE_ADDITIONAL_OPTIONS}
exit_if_error "CMake configuration failed for mc_rtc"
build_project mc_rtc
if $BUILD_TESTING
then
  test_project mc_rtc mc_rtc
fi
echo_log "-- [SUCCESS] Successfully built mc_rtc"

##############################
#  --  Build mc_rtc_ros  --  #
##############################
echo_log ""
echo_log "================================="
echo_log "== Building additional modules =="
echo_log "================================="
echo_log ""

if $WITH_ROS_SUPPORT
then
  build_catkin_workspace $CATKIN_WORKSPACE
fi

################################
#  --  Build extra modules  -- #
################################
echo_log "-- Building extra modules (robots, etc)"

if $WITH_HRP2
then
  echo_log "-- Installing with HRP2 robot support"
  if ! $WITH_ROS_SUPPORT
  then
    build_git_dependency git@gite.lirmm.fr:mc-hrp2/hrp2_drc_description
    echo_log "-- [OK] Successfully built the robot description $git_dep (no catkin)"
  fi
  build_git_dependency git@gite.lirmm.fr:mc-hrp2/mc-hrp2
  echo_log "-- [OK] Successfully built the robot module $git_dep"
fi

if $WITH_HRP4
then
  echo_log "-- Installing with HRP4 robot support"
  if ! $WITH_ROS_SUPPORT
  then
    build_git_dependency git@gite.lirmm.fr:mc-hrp4/hrp4
    echo_log "-- [OK] Successfully built the robot description $git_dep (no catkin)"
  fi
  build_git_dependency git@gite.lirmm.fr:mc-hrp4/mc-hrp4
  echo_log "-- [OK] Successfully built the robot module $git_dep"
fi

if $WITH_HRP4J
then
  echo_log "-- Installing with HRP4J robot support"
  if ! $WITH_ROS_SUPPORT
  then
    build_git_dependency git@gite.lirmm.fr:mc-hrp4/hrp4j_description
    echo_log "-- [OK] Successfully built the robot description $git_dep (no catkin)"
  fi
  build_git_dependency git@gite.lirmm.fr:mc-hrp4/mc_hrp4j
  echo_log "-- [OK] Successfully built the robot module $git_dep"
fi

if $WITH_HRP4CR
then
  echo_log "-- Installing with HRP4CR robot support"
  if ! $WITH_ROS_SUPPORT
  then
    build_git_dependency isri-aist/hrp4cr_description
    echo_log "-- [OK] Successfully built the robot description $git_dep (no catkin)"
  fi
  build_git_dependency isri-aist/mc_hrp4cr
  echo_log "-- [OK] Successfully built the robot module $git_dep"
fi

if $WITH_HRP5
then
  echo_log "-- Installing with HRP5 robot support"
  if ! $WITH_ROS_SUPPORT
  then
    build_git_dependency git@gite.lirmm.fr:mc-hrp5/hrp5_p_description
    echo_log "-- [OK] Successfully built the robot description $git_dep (no catkin)"
  fi
  build_git_dependency git@gite.lirmm.fr:mc-hrp5/mc_hrp5_p
  echo_log "-- [OK] Successfully built the robot module $git_dep"
fi

if $WITH_PANDA
then
  echo_log "-- Installing with PANDA robot support"
  build_git_dependency jrl-umi3218/mc_panda
  echo_log "-- [OK] Successfully built the robot module $git_dep"
fi

if $WITH_MC_UDP
then
  echo_log "-- Installing with mc_udp interface support"
  build_git_dependency_no_test jrl-umi3218/mc_udp $MC_UDP_INSTALL_PREFIX
  echo_log "-- [OK] Successfully built the interface $git_dep"
fi

if $WITH_MC_OPENRTM
then
  echo_log "-- Installing with mc_udp interface support"
  build_git_dependency_no_test jrl-umi3218/mc_openrtm $MC_OPENRTM_INSTALL_PREFIX
  echo_log "-- [OK] Successfully built the interface $git_dep"
fi

echo_log "-- [SUCCESS] All extra dedencencies have been installed"
echo_log "-- [SUCCESS] mc_rtc and the selected optional components have been successfully installed. Please read the following section."

echo_log ""
echo_log "=========================="
echo_log "== Installation success =="
echo_log "=========================="
echo "-- Installation log has been written to $BUILD_LOGFILE"
echo_log ""
echo_log "Please add the following lines to your .bashrc/.zshrc"
echo_log ""
if [[ $OSTYPE == "darwin"* ]]
then
  echo_log "export PATH=$INSTALL_PREFIX/bin:\$PATH"
  echo_log "export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$DYLD_LIBRARY_PATH"
  echo_log "export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH"
  echo_log "export PYTHONPATH=$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages:\$PYTHONPATH"
else
  echo_log "export PATH=$INSTALL_PREFIX/bin:\$PATH"
  echo_log "export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$LD_LIBRARY_PATH"
  echo_log "export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH"
  echo_log "export PYTHONPATH=$INSTALL_PREFIX/lib/python$PYTHON_VERSION/site-packages:\$PYTHONPATH"

  if $WITH_ROS_SUPPORT
  then
    echo_log "source $CATKIN_WORKSPACE/devel/setup.bash"
    echo_log ""
    echo_log "If you are running zsh, replace setup.bash with setup.zsh in that last line"
  fi
fi

echo_log ""
echo_log "If you want autocompletion on the scripts add also the following to your .bashrc/.zshrc"
echo_log "source $this_dir/autocompletion.bash"
echo_log "If you are running zsh, replace autocompletion.bash with autocompletion.zsh in that last line"

if $WITH_PANDA
then
  echo_log ""
  echo_log "== Panda robot =="
  echo_log "The Panda robot module has been installed, you will be able to run controllers and simulations."
  echo_log "To execute controllers on the real robot you will need to install mc_franka on a real-time linux system"
  echo_log "See https://github.com/jrl-umi3218/mc_franka for instructions"
fi
