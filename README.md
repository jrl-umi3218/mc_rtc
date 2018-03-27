Introduction
==

`mc_rtc` is an interface for simulation and robot control systems. These systems should provide the state of a given robot (joints' values, sensor readings...) and in return `mc_rtc` will provide the desired robot's state (command). This is done through the `mc_control::MCGlobalController` class. This class does not perform control by itself but rather delegates this task to the `mc_control::MCController` derived objects that it holds. Writing a controller within the `mc_rtc` framework is done by writing a class that inherits from the `mc_control::MCController` base class and implements the required functionnality. The details of this process can be found in a later section of this document.

`mc_rtc` is a super-set of SpaceVecAlg/RBDyn/Tasks libraries which provides a friendlier/easier interface for the user. It was originally a C++ portage of the [mc_ros](https://gite.lirmm.fr/multi-contact/mc_ros) package aimed at running the controller on the embedded computer of the robot. As such, readers familiar with the Python interface proposed in `mc_ros` should easily find their way around `mc_rtc`.

This document assumes some familiarity with the [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg), [RBDyn](https://github.com/jrl-umi3218/RBDyn) and [Tasks](https://github.com/jrl-umi3218/Tasks) although the libraries in the `mc_rtc` package abstract most of these libraries inner-workings. Unfamiliar users may refer to the [SpaceVecAlg and RBDyn tutorials](https://github.com/jorisv/sva_rbdyn_tutorials) and the [SpaceVecAlg and RBDyn presentations](https://github.com/jorisv/sva_rbdyn_presentation) authored by Joris Vaillant.

The [project wiki](https://gite.lirmm.fr/multi-contact/mc_rtc/wikis/home) provides a somewhat complete introduction of the framework functionalities.

Feel free to [open an issue](https://gite.lirmm.fr/multi-contact/mc_rtc/issues/new?issue) if you have any questions or feature requests.

Available interfaces
==

The following interface between `mc_rtc` and 3rd-party framework are available:
- [mc_rtc_ros](https://gite.lirmm.fr/multi-contact/mc_rtc_ros) contains a number of tools related to the framework as well as a simple "ticker" that allows to run controllers in an open-loop fashion;
- [mc_openrtm](https://gite.lirmm.fr/multi-contact/mc_openrtm) embeds `mc_rtc` into a RTC component. Can be used to control a robot that is using the OpenRTM framework;
- [mc_vrep](https://gite.lirmm.fr/multi-contact/mc_vrep) interface with the VREP simulation software;
- [mc_rtc_nao](https://gite.lirmm.fr/multi-contact/mc_rtc_naoqi) interface with the NAOqi framework (Softbank Robotics robots);
