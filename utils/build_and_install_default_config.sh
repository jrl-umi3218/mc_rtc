#default settings
export SOURCE_DIR=`cd $(dirname $0)/../../; pwd`
export BUILD_SUBDIR=build
export INSTALL_PREFIX="/usr/local"
export WITH_ROS_SUPPORT="true"
export WITH_PYTHON_SUPPORT="true"
export PYTHON_USER_INSTALL="false"
export PYTHON_FORCE_PYTHON2="false"
export PYTHON_FORCE_PYTHON3="false"
export PYTHON_BUILD_PYTHON2_AND_PYTHON3="false"
export WITH_LSSOL="false"
export WITH_HRP2="false"
export WITH_HRP4="false"
export WITH_HRP4J="false"
export WITH_HRP4CR="false"
export WITH_HRP5="false"
export WITH_MC_OPENRTM="false"
export MC_OPENRTM_INSTALL_PREFIX="$INSTALL_PREFIX"
export WITH_MC_UDP="false"
export MC_UDP_INSTALL_PREFIX="$INSTALL_PREFIX"
export BUILD_TYPE="RelWithDebInfo"
export BUILD_TESTING="true"
export BUILD_BENCHMARKS="false"
export INSTALL_SYSTEM_DEPENDENCIES="true"
export CLONE_ONLY="false"
export SKIP_UPDATE="false"
# This configuration option lets the script choose what to do when local git repositories are in
# an unclean state (have local changes). The default false will stop the script with an error.
# If true, the repository will be compiled as-is without trying to fetch the remote changes.
export SKIP_DIRTY_UPDATE="false"
if command -v nproc > /dev/null
then
   export BUILD_CORE=`nproc`
else
   export BUILD_CORE=`sysctl -n hw.ncpu`
fi
export CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_CORE}
export LOG_PATH="/tmp"
export BUILD_LOGFILE="$LOG_PATH/build_and_install_warnings-`date +%Y-%m-%d-%H-%M-%S`.log"
export ASK_USER_INPUT="true"
export TEE=`which tee`
export ALLOW_ROOT="false"
