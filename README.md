mc_rtc
======

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)

Introduction
------------

`mc_rtc` is an interface for simulation and robot control systems. These systems should provide the state of a given robot (joints' values, sensor readings...) and in return `mc_rtc` will provide the desired robot's state (command). This is done through the `mc_control::MCGlobalController` class. This class does not perform control by itself but rather delegates this task to the `mc_control::MCController` derived objects that it holds. Writing a controller within the `mc_rtc` framework is done by writing a class that inherits from the `mc_control::MCController` base class and implements the required functionnality. The details of this process can be found in a later section of this document.

`mc_rtc` is a super-set of SpaceVecAlg/RBDyn/Tasks libraries which provides a friendlier/easier interface for the user. It was originally a C++ portage of the [mc_ros](https://gite.lirmm.fr/multi-contact/mc_ros) package aimed at running the controller on the embedded computer of the robot. As such, readers familiar with the Python interface proposed in `mc_ros` should easily find their way around `mc_rtc`.

This document assumes some familiarity with the [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg), [RBDyn](https://github.com/jrl-umi3218/RBDyn) and [Tasks](https://github.com/jrl-umi3218/Tasks) although the libraries in the `mc_rtc` package abstract most of these libraries inner-workings. Unfamiliar users may refer to the [SpaceVecAlg and RBDyn tutorials](https://github.com/jorisv/sva_rbdyn_tutorials) and the [SpaceVecAlg and RBDyn presentations](https://github.com/jorisv/sva_rbdyn_presentation) authored by Joris Vaillant.

The [project wiki](https://gite.lirmm.fr/multi-contact/mc_rtc/wikis/home) provides a somewhat complete introduction of the framework functionalities.

Feel free to [open an issue](https://gite.lirmm.fr/multi-contact/mc_rtc/issues/new?issue) if you have any questions or feature requests.

Available interfaces
--------------------

The following interface between `mc_rtc` and 3rd-party framework are available:
- [mc_rtc_ros](https://gite.lirmm.fr/multi-contact/mc_rtc_ros) contains a number of tools related to the framework as well as a simple "ticker" that allows to run controllers in an open-loop fashion;
- [mc_openrtm](https://gite.lirmm.fr/multi-contact/mc_openrtm) embeds `mc_rtc` into a RTC component. Can be used to control a robot that is using the OpenRTM framework;
- [mc_vrep](https://gite.lirmm.fr/multi-contact/mc_vrep) interface with the VREP simulation software;
- [mc_rtc_nao](https://gite.lirmm.fr/multi-contact/mc_rtc_naoqi) interface with the NAOqi framework (Softbank Robotics robots);

Installing
----------

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
 * [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)
 * [RBDyn](https://github.com/jrl-umi3218/RBDyn)
 * [eigen-qld](https://github.com/jrl-umi3218/eigen-qld)
 * [eigen-lssol]() (Optional, if you have the LSSOL licence ask us this library)
 * [sch-core](https://github.com/jrl-umi3218/sch-core)
 * [Tasks](https://github.com/jrl-umi3218/Tasks)
 * [mc_rbdyn_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf)
 * [mc_rtc_ros_data](https://gite.lirmm.fr/multi-contact/mc_rtc_ros_data)

If you wish to get Python bindings you will also need the following:
 * [Cython](http://cython.org/) >= 0.2
 * [python-pip]()
 * [python-numpy]()
 * [python-nose]()
 * [python-coverage]()
 * [python-git]() (pip name: `GitPython`)
 * [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython)
 * [sch-core-python](https://github.com/jrl-umi3218/sch-core-python)

If you are building benchmarks you will also need:
 * [benchmark](https://github.com/google/benchmark)

mc_rtc also uses the following great 3rd party libraries:

 * [Qhull](http://www.qhull.org/) under the [Qhull license](http://www.qhull.org/COPYING.txt)
 * [RapidJSON](http://rapidjson.org/) under the [MIT license](https://github.com/Tencent/rapidjson/blob/master/license.txt)
 * [mpack](https://github.com/ludocode/mpack) under the [MIT license](https://github.com/ludocode/mpack/blob/develop/LICENSE)
