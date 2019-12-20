---
layout: tutorials
install_apt:
  - name: Install stable version
    lang: bash
    source: |
      # Make sure you have required tools
      sudo apt install apt-transport-https lsb-release ca-certificates gnupg
      # Add our key
      sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 892EA6EE273707C6495A6FB6220D644C64666806
      # Add our repository
      sudo sh -c 'echo "deb https://dl.bintray.com/gergondet/multi-contact-release $(lsb_release -sc) main" | \
      sudo tee /etc/apt/sources.list.d/multi-contact.list'
      # Update packages list
      sudo apt update
      # Install packages
      sudo apt install libmc-rtc-dev mc-rtc-utils python3-mc-rtc
      # Assuming you have a ROS distribution mirror setup
      sudo apt install ros-${ROS_DISTRO}-mc-rtc-plugin
  - name: Install head version
    lang: bash
    source: |
      # Make sure you have required tools
      sudo apt install apt-transport-https lsb-release ca-certificates gnupg
      # Add our key
      sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 892EA6EE273707C6495A6FB6220D644C64666806
      # Add our repository
      sudo sh -c 'echo "deb https://dl.bintray.com/gergondet/multi-contact-head $(lsb_release -sc) main" | \
      sudo tee /etc/apt/sources.list.d/multi-contact.list'
      # Update packages list
      sudo apt update
      # Install packages
      sudo apt install libmc-rtc-dev mc-rtc-utils python3-mc-rtc
      # Assuming you have a ROS distribution mirror setup
      sudo apt install ros-${ROS_DISTRO}-mc-rtc-plugin
---

## Overview

mc\_rtc is an interface for simulation and robot control systems. These systems should provide the state of a given robot (joints' values, sensor readings...) and in return mc\_rtc will provide the desired robot's state (command). This is done through the `mc_control::MCGlobalController` class. This class does not perform control by itself but rather delegates this task to the `mc_control::MCController` derived objects that it holds. Writing a controller within the mc\_rtc framework is done by writing a class that inherits from the `mc_control::MCController` base class and implements the required functionnality. We implement such a controller in the following tutorials. The present tutorial simply explains how to build/install the framework on your machine.

<img src="img/mc_rtc_architecture.jpg" alt="architecture_overview" class="img-fluid" />

## Installation instruction

We provide binary installation for current Ubuntu LTS releases. We also provide a source release using an easy-to-use script for Ubuntu, macOS and Windows.

### Ubuntu LTS (16.04, 18.04, 20.04)

{% include show_sources.html sources=page.install_apt copy=false id="install_apt" %}

*Note: the distributed version of mc\_rtc runs with the QLD QP solver through [eigen-qld](https://github.com/jrl-umi3218/eigen-qld). If you have access to the LSSOL solver and thus can install [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol) then you can build [Tasks](https://github.com/jrl-umi3218/Tasks) with LSSOL support and install it in `/usr`. The two versions are binary compatible.*

### Building from source (script)

If you are building on Linux or macOS you can skip to the Building section. However, Windows users have to follow extra-instructions.

#### Windows pre-requisites

If you are building on Windows, you need to have the following tools installed before starting:
- [Visual Studio 2019](https://visualstudio.microsoft.com/vs/) - when installing, ensure that you select the Python extension if you wish to build the Python bindings. Furthermore, you should make sure that Visual Studio's `python` and `pip` executables are in your `PATH` environment variable;
- [Git Bash](https://git-scm.com/download/win) - we will use this tool to clone mc\_rtc and start the installation script;
- [CMake](https://cmake.org/download/) - install the latest version available;
- [Boost](https://www.boost.org/) - install the latest binaries avaible from [sourceforce](https://sourceforge.net/projects/boost/files/boost-binaries/). Make sure to select the right version for your computer and Visual Studio version, for example, for Boost 1.72 and Visual Studio 2019 on a 64 bits computer it should be: [boost_1_72_0-msvc-14.2-64.exe](https://sourceforge.net/projects/boost/files/boost-binaries/1.72.0/boost_1_72_0-msvc-14.2-64.exe/download). After installing, make sure to set the environment variable `BOOST_ROOT` to the location where you installed Boost and add Boost DLLs path to your `PATH` environment. For example, if you installed into `C:\local\boost_1_72_0` then `BOOST_ROOT` should be `C:\local\boost_1_72_0` and `%BOOST_ROOT%\lib64-msvc-14.2` should be in your `PATH`;
- [mingw-w64](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) - this will be used to compile Fortran code on Windows. After downloading, you can run the executable. At one point, it will ask you to select several options: Version, Architecture, Threads, Exception and Build revision. For Version you can choose the most recent one (default), for Architecture you should choose x86\_64 if you are building on a 64 bits Windows (likely), for Threads you should choose win32, for Exceptions you should choose seh (default). After installing, you should make sure the `bin` folder of the installation is in your `PATH` environment, e.g. `C:\mingw-w64\x86_64-8.1.0-release-win32-seh-rt_v6-rev0\mingw64\bin`

Note: it should also work and compile with Visual Studio 2017. However, only Visual Studio 2019 is regularly tested

#### Building

1. Clone the [mc\_rtc](https://github.com/jrl-umi3218/mc_rtc) repository;
2. Go into the mc\_rtc directory and update submodules `git submodule update --init`;
3. Go into the `utils` directory and locate the file named `build_and_install.sh`;
4. Edit some of the options to your liking: `INSTALL_PREFIX`, `WITH_ROS_SUPPORT`, `ROS_DISTRO`. On Ubuntu, ROS will be installed if you enable ROS support and it was not already installed. Otherwise, you are required to install ROS by yourself before attempting to install mc\_rtc with ROS support;
5. Run `./build_and_install.sh`

The script will take care of installing the required dependencies, clone all required source codes, build and install them. This may take a while.

If the script fails, please open up an issue on mc\_rtc issue tracker, providing the following information:
- System (compiler, distribution/OSX version)
- Script output
- Any detail you might think relevant

Once the script has succeeded, you are finished. You can jump to the next [section]({{site.baseurl}}/tutorials/introduction/configuration.html).

### Building from source (no script)

Building from sources on other platforms is not well documented at the moment. If you wish to embark on this adventure, the following packages are required to build mc\_rtc:
- [CMake](https://cmake.org/) >= 3.1
- [Boost](https://www.boost.org/) >= 1.49
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2
- [tinyxml2](https://github.com/leethomason/tinyxml2)
- [GEOS](https://trac.osgeo.org/geos) (With C++ bindings)
- [LTDL](https://www.gnu.org/software/libtool/manual/html_node/Libltdl-interface.html) (Not required for Windows users)
- [nanomsg](https://github.com/nanomsg/nanomsg)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [hpp-spline](https://github.com/humanoid-path-planner/hpp-spline)
- [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)
- [RBDyn](https://github.com/jrl-umi3218/RBDyn)
- [eigen-qld](https://github.com/jrl-umi3218/eigen-qld)
- [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol)
- [sch-core](https://github.com/jrl-umi3218/sch-core)
- [Tasks](https://github.com/jrl-umi3218/Tasks)
- [mc\_rbdyn\_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf)
- [mc_rtc_data](https://github.com/jrl-umi3218/mc_rtc_data)

mc\_rtc also has a ROS plugin that enables automated robot's status publication as ROS topics and facilitate the integration with ROS tools (e.g. RViZ), to build this you will need:

 * [mc_rtc_msgs](https://github.com/jrl-umi3218/mc_rtc_msgs)

If you wish to get Python bindings you will also need the following:
 * [Cython](http://cython.org/) >= 0.2
 * [python-pip]()
 * [python-numpy]()
 * [python-nose]()
 * [python-coverage]()
 * [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython)
 * [sch-core-python](https://github.com/jrl-umi3218/sch-core-python)

Additional Python libraries are required to run mc\_rtc tools:
 * [python-git]() (pip name: `GitPython`)
 * [python-pyside]()

The following packages are not required but bring additionnal features:
- [ROS](http://www.ros.org/)
- [mc\_rtc\_ros](https://github.com/jrl-umi3218/mc_rtc_ros)

If `roscpp` is available during build then `tf2_ros` and `sensor_msgs` are also required. This will add some integration between the `ROS` framework and `mc_rtc` (e.g. robot's state publication) and allow controllers to use ROS functionnalities (provides a `ros::NodeHandle` instance that can be used anywhere in `mc_rtc`).
