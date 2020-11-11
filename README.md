mc_rtc
======

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/mc_rtc/workflows/CI%20of%20mc_rtc/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_rtc/actions?query=workflow%3A%22CI+of+mc_rtc%22)
[![Website](https://img.shields.io/badge/website-online-brightgreen?logo=read-the-docs&style=flat)](https://jrl-umi3218.github.io/mc_rtc/)

Introduction
------------

`mc_rtc` is an interface for simulation and robot control systems. These systems should provide the state of a given robot (joints' values, sensor readings...) and in return `mc_rtc` will provide the desired robot's state (command). This is done through the `mc_control::MCGlobalController` class. This class does not perform control by itself but rather delegates this task to the `mc_control::MCController` derived objects that it holds. Writing a controller within the `mc_rtc` framework is done by writing a class that inherits from the `mc_control::MCController` base class and implements the required functionnality. The details of this process can be found in the project documentation.

`mc_rtc` is a super-set of SpaceVecAlg/RBDyn/Tasks libraries which provides a friendlier/easier interface for the user.

This framework assumes some familiarity with the [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg), [RBDyn](https://github.com/jrl-umi3218/RBDyn) and [Tasks](https://github.com/jrl-umi3218/Tasks) although the libraries in the `mc_rtc` package abstract most of these libraries inner-workings. Unfamiliar users may refer to the [SpaceVecAlg and RBDyn tutorials](https://github.com/jorisv/sva_rbdyn_tutorials) and the [SpaceVecAlg and RBDyn presentations](https://github.com/jorisv/sva_rbdyn_presentation) authored by Joris Vaillant.

The [project website](https://jrl-umi3218.github.io/mc_rtc/) provides a somewhat complete introduction of the framework functionalities.

Feel free to [open an issue](https://github.com/jrl-umi3218/mc_rtc/issues/new) for questions, feature requests or problems you might encounter.

Available interfaces
--------------------

The following interface between `mc_rtc` and 3rd-party framework are available:
- [mc_rtc_ros](https://github.com/jrl-umi3218/mc_rtc_ros) contains a number of tools related to the framework as well as a simple "ticker" that allows to run controllers in an open-loop fashion;
- [mc_openrtm](https://github.com/jrl-umi3218/mc_openrtm) embeds `mc_rtc` into a RTC component, this can be used to control a robot that is using the OpenRTM framework, an example with the virtual JVRC1 robot is also provided;
- [mc_udp](https://github.com/jrl-umi3218/mc_udp) provides a thin UDP-based client/server. The client is a regular `mc_rtc` interface that gets data through the UDP socket. The server can easily be implemented to interface with your robot. A server to interface with OpenRTM components is provided.
- [mc_vrep](https://github.com/jrl-umi3218/mc_vrep) interface with the VREP simulation software;
- [mc_rtc_naoqi](https://github.com/jrl-umi3218/mc_rtc_naoqi) interface with the NAOqi framework (Softbank Robotics robots);

Installing
----------

Please see [mc_rtc installation guide](https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html) for details.

mc\_rtc also uses the following great 3rd party libraries but does not require you to get or install them:

 * [Qhull 2020.2](http://www.qhull.org/) under the [Qhull license](http://www.qhull.org/COPYING.txt)
 * [RapidJSON](http://rapidjson.org/) under the [MIT license](https://github.com/Tencent/rapidjson/blob/master/license.txt)
 * [mpack](https://github.com/ludocode/mpack) under the [MIT license](https://github.com/ludocode/mpack/blob/develop/LICENSE)
