mc_rtc
======

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/mc_rtc/workflows/CI%20of%20mc_rtc/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_rtc/actions?query=workflow%3A%22CI+of+mc_rtc%22)
[![Website](https://img.shields.io/badge/website-online-brightgreen?logo=read-the-docs&style=flat)](https://jrl-umi3218.github.io/mc_rtc/)

Introduction
------------

`mc_rtc` is an interface for simulation and robot control systems. These systems should provide the state of a given robot (joints' values, sensor readings...) and in return `mc_rtc` will provide the desired robot's state (command). This is done through the `mc_control::MCGlobalController` class. This class does not perform control by itself but rather delegates this task to the `mc_control::MCController` derived objects that it holds. Writing a controller within the `mc_rtc` framework is done by writing a class that inherits from the `mc_control::MCController` base class and implements the required functionnality. The details of this process can be found in the project documentation.

`mc_rtc` is a super-set of SpaceVecAlg/RBDyn/Tasks libraries which provides a friendlier/easier interface for the user.

This document assumes some familiarity with the [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg), [RBDyn](https://github.com/jrl-umi3218/RBDyn) and [Tasks](https://github.com/jrl-umi3218/Tasks) although the libraries in the `mc_rtc` package abstract most of these libraries inner-workings. Unfamiliar users may refer to the [SpaceVecAlg and RBDyn tutorials](https://github.com/jorisv/sva_rbdyn_tutorials) and the [SpaceVecAlg and RBDyn presentations](https://github.com/jorisv/sva_rbdyn_presentation) authored by Joris Vaillant.

The [project website](https://jrl-umi3218.github.io/mc_rtc/) provides a somewhat complete introduction of the framework functionalities.

Feel free to [open an issue](https://github.com/jrl-umi3218/mc_rtc/issues/new) for questions, feature requests or problems you might encounter.

Available interfaces
--------------------

The following interface between `mc_rtc` and 3rd-party framework are available:
- [mc_rtc_ros](https://github.com/jrl-umi3218/mc_rtc_ros) contains a number of tools related to the framework as well as a simple "ticker" that allows to run controllers in an open-loop fashion;
- [mc_openrtm](https://github.com/jrl-umi3218/mc_openrtm) embeds `mc_rtc` into a RTC component, this can be used to control a robot that is using the OpenRTM framework, an example with the virtual JVRC1 robot is also provided;
- [mc_vrep](https://github.com/jrl-umi3218/mc_vrep) interface with the VREP simulation software;
- [mc_rtc_naoqi](https://github.com/jrl-umi3218/mc_rtc_naoqi) interface with the NAOqi framework (Softbank Robotics robots);

Installing
----------

## Ubuntu LTS (16.04, 18.04, 20.04)

```bash
# Make sure you have required tools
sudo apt install apt-transport-https lsb-release ca-certificates gnupg
# Add our key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 892EA6EE273707C6495A6FB6220D644C64666806
# Add our repository (stable versions)
sudo sh -c 'echo "deb https://dl.bintray.com/gergondet/multi-contact-release $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/multi-contact.list'
# Use this to setup the HEAD version
# sudo sh -c 'echo "deb https://dl.bintray.com/gergondet/multi-contact-head $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/multi-contact.list'
# Update packages list
sudo apt update
# Install packages
sudo apt install libmc-rtc-dev mc-rtc-utils python3-mc-rtc
# Assuming you have a ROS distribution mirror setup
sudo apt install ros-${ROS_DISTRO}-mc-rtc-plugin
```

Note: the distributed version of mc\_rtc runs with the QLD QP solver through [eigen-qld](https://github.com/jrl-umi3218/eigen-qld). If you have access to the LSSOL solver and thus can install [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol) then you can build [Tasks](https://github.com/jrl-umi3218/Tasks) with LSSOL support and install it in `/usr`. The two versions are binary compatible.

## Build from source

### Using a script (Ubuntu/MacOS)

After cloning the repository, cd into the utils directory then:

```bash
./build_and_install.sh --help
# Adjust options according to your needs
./build_and_install.sh
```

The script can also be used to update your installation.

### From sources

#### Dependencies

To compile you need the following tools and libraries:

 * [CMake](https://cmake.org/) >= 3.1
 * [Boost](https://www.boost.org/) >= 1.49
 * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2
 * [tinyxml2](https://github.com/leethomason/tinyxml2)
 * [GEOS](https://trac.osgeo.org/geos) (With C++ bindings)
 * [LTDL](https://www.gnu.org/software/libtool/manual/html_node/Libltdl-interface.html) (Not required for Windows users)
 * [nanomsg](https://nanomsg.org/)
 * [yaml-cpp](https://github.com/jbeder/yaml-cpp)
 * [hpp-spline](https://github.com/humanoid-path-planner/hpp-spline)
 * [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)
 * [RBDyn](https://github.com/jrl-umi3218/RBDyn)
 * [eigen-qld](https://github.com/jrl-umi3218/eigen-qld)
 * [eigen-lssol]() (Optional, if you have the LSSOL licence ask us this library)
 * [sch-core](https://github.com/jrl-umi3218/sch-core)
 * [Tasks](https://github.com/jrl-umi3218/Tasks)
 * [mc_rbdyn_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf)
 * [mc_rtc_data](https://github.com/jrl-umi3218/mc_rtc_data)

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

If you are building benchmarks you will also need:
 * [benchmark](https://github.com/google/benchmark)

mc\_rtc also uses the following great 3rd party libraries but does not require you to get or install them:

 * [Qhull](http://www.qhull.org/) under the [Qhull license](http://www.qhull.org/COPYING.txt)
 * [RapidJSON](http://rapidjson.org/) under the [MIT license](https://github.com/Tencent/rapidjson/blob/master/license.txt)
 * [mpack](https://github.com/ludocode/mpack) under the [MIT license](https://github.com/ludocode/mpack/blob/develop/LICENSE)
