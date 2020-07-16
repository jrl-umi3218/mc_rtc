####################################################
# build_and_install.sh user configuration settings #
####################################################
# The option defined here override the corresponding default values
# defined in build_and_install_default_config.sh script
# Please uncomment and edit the desired options.
###################################################

# Directory where mc_rtc dependencies will be cloned
# Defaults to the parent directory of the mc_rtc repository folder
# export SOURCE_DIR=`cd $(dirname $0)/../; pwd`

# Path in which mc_rtc and its dependencies will be installed
# Note if different from /usr/local, make sure to appropriately set the paths variables
# The script will automatically inform you of the required paths after successful installation.
# export INSTALL_PREFIX="/usr/local"

##
# Build type.
# - For performance on the robots use "Release"
# - For normal development, "RelWithDebInfo" should be fast enough on most machines
##
# export BUILD_TYPE="RelWithDebInfo"

##
# Number of parallel builds. Default: automatically detected
##
# export CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_CORE}

##
# When true, installs the required global system dependencies (APT on ubuntu)
##
# export INSTALL_SYSTEM_DEPENDENCIES="true"

##
# Whether to build with ROS support
# On Ubuntu, ROS will be installed if you enable ROS support and it was not already installed. Otherwise, you are required to install ROS by yourself before attempting to install mc_rtc with ROS support;
##
#export WITH_ROS_SUPPORT="true"

##
# Python settings
##
# export WITH_PYTHON_SUPPORT="true"
# export PYTHON_USER_INSTALL="false"
# export PYTHON_FORCE_PYTHON2="false"
# export PYTHON_FORCE_PYTHON3="false"
# export PYTHON_BUILD_PYTHON2_AND_PYTHON3="false"

##
# These repositories require additional permissions.
# If you're allowed access, you can set those to true
##
# export WITH_LSSOL="false"
# export WITH_HRP2="false"
# export WITH_HRP4="false"
# export WITH_HRP4J="false"
# export WITH_HRP5="false"

##
# Interfaces -- allows communication with the choreonoid simulator, and with the HRP robots.
# Both mc_udp and mc_openrtm require hrpsys-base and its dependencies to be installed. The script does not
# automatically install these dependencies, please install them manually before running the script.
# In case choreonoid was installed in a different path than $INSTALL_PREFIX, set
# MC_UDP_INSTALL_PREFIX and MC_OPENRTM_INSTALL_PREFIX to choreonoid's base installation path
##
# export WITH_MC_OPENRTM="false"
# export MC_OPENRTM_INSTALL_PREFIX="$INSTALL_PREFIX"
# export WITH_MC_UDP="false"
# export MC_UDP_INSTALL_PREFIX="$INSTALL_PREFIX"

##
# For additional options, please refer to build_and_install_default_config.sh
##
