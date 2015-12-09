Introduction
==

`mc_rtc` is a super-set of SpaceVecAlg/RBDyn/Tasks libraries which provides a friendlier/easier interface for the user. It was originally a C++ portage of the [mc_ros](https://gite.lirmm.fr/multi-contact/mc_ros) package aimed at running the controller on the embedded computer of the robot, as such, readers familiar with the Python interface proposed in `mc_ros` should easily find their way around `mc_rtc`.

This document assumes some familiarity with the [SpaceVecAlg](https://github.com/jorisv/SpaceVecAlg), [RBDyn](https://github.com/jorisv/RBDyn) and [Tasks](https://github.com/jorisv/Tasks) although the libraries in the `mc_rtc` package abstract most of these libraries inner-workings. Unfamiliar users may refer to the [SpaceVecAlg and RBDyn tutorials](https://github.com/jorisv/sva_rbdyn_tutorials) and the [SpaceVecAlg and RBDyn presentations](https://github.com/jorisv/sva_rbdyn_presentation) authored by Joris Vaillant.

Questions specific to the `mc_rtc` package can be adressed to <pierre.gergondet@gmail.com>. Some frequently asked questions are answered in the last section of this document.

Building
==

The following packages are required to build mc_rtc:
- `mc_non_ros` (see the [installation wiki](https://gite.lirmm.fr/multi-contact/installation_wiki/wikis/home) for more information)
- [mc_rbdyn_urdf_cpp](https://gite.lirmm.fr/multi-contact/mc_rbdyn_urdf_cpp)
- [jsoncpp](https://github.com/open-source-parsers/jsoncpp) (or `libjsoncpp-dev` on debian-like systems)
- [geos](https://github.com/libgeos/libgeos) (or `libgeos++-dev` on debian-like systems)o

The following packages are not required but bring additionnal features:
- [ROS](http://www.ros.org/)
> If `roscpp` is available during build then `tf2_ros` and `sensor_msgs` are also required. This will add some integration between the `ROS` framework and `mc_rtc` (e.g. robot's state publication) and allow controllers to use ROS functionnalities (provides a `ros::NodeHandle` instance that can be used anywhere in `mc_rtc`).
- [hrpsys-base](https://github.com/fkanehiro/hrpsys-base)
> If `hrpsys-base` is available during build then the RTC component `MCControl` will be built. This is necessary to use `mc_rtc` in the `choreonoid` simulator and on the robot.

Work in progress (last update 2015/12/08)
==

- python bindings using cython ([current progress](https://gite.lirmm.fr/multi-contact/mc_cython)) aimed at replacing most of `mc_ros`
- additions to `mc_control` to allow to define new controllers outside of `mc_rtc`
- additions to `mc_robots` to allow to define new "complex" robots outside of `mc_rtc`
- unify remote interactions with the controller(s)

Library by library overview
==

While `mc_rtc` is a single package, it consists of 7 independent libraries. We give an overview of each library and their role here.

mc_rbdyn
--

A super-set of `SpaceVecAlg` and `RBDyn`. Essentially turns collections of low-level types into `Robot`, `Surface`, `Contact`, etc...
 Refer to the headers/doxygen documentation for available class/methods.

### A note on the `Robot` class

`Tasks` is designed to operate with a multi-robot approach in mind (e.g. a door may be seen as a robot with 2 degrees of freedom) and as such most classes in `mc_rtc` will operate on the `Robots` class rather than the `Robot` class to "talk" with `Tasks`. In order to maintain a coherent state, this creates a tight coupling between the `Robot` and `Robots` classes. Therefore, the `Robot` class cannot exist by itself and only the `Robots` class should be stored.

mc_robots
--

Implementations of the `mc_rbdyn::RobotModule`, in particular:
- `EnvRobotModule` is able to load any non-interactive robot (i.e. a static environment) that follows the `mc_env_description` scheme
- `HRP2DRCRobotModule` and `HRP2DRCGripperRobotModule` provide a HRP2-kai robot (without/with grippers in the model respectively)

mc_solver
--

Provides a high-level interface above `tasks::qp::QPSolver` and `Tasks` constraints that can be used with `mc_rbdyn` types. See the headers/doxygen documentation for available class/methods.

mc_tasks
--

A mixture of:
- High-level interface above some `Tasks` tasks that can be used with `mc_rbdyn` types
- Higher-level tasks (e.g. `AddRemoveContactTask`)

See the headers/doxygen documentation for available class/methods.

mc_trajectory (experimental)
--

Set of functions/class to generate trajectories

mc_control
--

Where the controllers are defined, more details regarding the implementation of a new controller are provided below but all controllers should inherit from the `MCVirtualController` class.

Existing controllers include:
- `PostureController` controls a robot's posture
- `Body6dController` controls a robot's end-effector(s)
- `CoMController` controls a robot's CoM
- `SeqController` implements a FSM to play a sequence of stance/actions obtained from planning softwares

The `MCGlobalController` class is used to manage the controllers and transfer the information from the simulation/robot (e.g. force sensors' data or joints positions) to the current controller.

mc_rtc_ros
--

If `mc_rtc` was compiled with `ROS` support then this library is used by `MCGlobalController` to communicate with `ROS`. It can also be invoked to retrieve a `ros::NodeHandle` instance in any class of `mc_rtc` that may require it for ROS functionnalities.


Implementing a new robot module
==

### A note on environments

To import a new non-interactive robot (i.e. an environment) it is not necessary to define a new robot inside or outisde mc_rtc as the `mc_robots::EnvRobotModule` should work out of the box given the proper environment path and name as long as the environment definition follows the `mc_env_description` scheme.

Inside mc_rtc
--

1. Write a new class that inherits from `mc_rbdyn::RobotModule` (e.g. `include/mc_robots/MyRobot.h` and `src/mc_robots/MyRobot.cpp`)
2. Implement the virtual functions your robot requires
3. Add your source files to the `mc_robots_SRC` variable in `src/CMakeLists.txt`
4. Your robot can now be used in `mc_rtc`

Outisde mc_rtc
--

Not done yet. Should be similar to the previous case with the addition of external create/delete functions.

Implementing a new controller
==

Inside mc_rtc
--

1. Choose a controller class to inherit from, there is 4 possible choices here. (*NB* It is fairly likely that option 1, 2 and 3 will be merged in the future)
  1. `MCVirtualController`: this is the most basic class and as such many functions have to be re-implemented
  2. `MCController`: implements a single robot controller and defines common constraints for it plus a posture task (none of which are put into the solver initially, this is up to the user)
  3. `MCMRQPController`: implements a multi-robot controller. The controller considers that the first robot is the "main" robot (typically, `MCDrivingController` is such a controller where the main robot is HRP-2 and the second robot is the car which has only one dof) and defines a set of constraints for this robot.
  4. Derive from any other controller
2. Add the tasks/constraints you need, implement the functions you require, implement the services you provide
3. Include your controller header in `mc_control/mc_global_controller.h` and add `std::shared_ptr` instance of your controller to the members of `MCGlobalController`
4. Handle the creation of the controller in `MCGlobalController` constructor
5. (Optionnal) Implement controller switch from a service call (*NB* very optionnal as this is likely to become obsolete and is really cumbersome to do atm)

###Functions to implement

A quick description of the `virtual` functions in `MCVirtualController` and what they are supposed to do.

####`virtual void reset(const ControllerResetData & reset_data)`

Called when:
1. the control loop starts
2. `MCGlobalController` switches from a different controller

ATM, the `ControllerResetData` holds the following data: (depending on the moment of the invokation)
1. "True" state of the robot (encoders)
2. Controlled state of the robot forwarded by the previous controller

####`virtual bool run()`

Called at every iteration of the control loop. Runs in the real-time context on the robot. Returns `false` to indicate a failure, `true` otherwise.

####`virtual const QPResultMsg & send(const double & t)`

Returns the current control output. Does not run if `run()` invokation failed.


####`bool read_msg(std::string & msg)` and `bool read_write_msg(std::string & msg, std::string & out)`

These functions are used to provide a generic service interface. In both functions, `msg` holds the content of the request made by the caller. In the `read_write_msg`, `out` may be filled with an answer for the caller. Both functions should return `false` to indicate that the controller was not able to handle the request `msg` and `true` otherwise.

Outisde mc_rtc
--

Not done yet. Steps 1 and 2 are likely to remain identical while 3 and 4 will be streamlined.

A note on real-time
--

When writting a controller that will ultimately run as part of the real-time loop on the robot one has to be careful about potential performance bottleneck as missing real-time iterations will result in non-smooth behaviours. In particular, **everything that happens in the `run` method is happenning in the real-time loop** and thus one should avoid:
- io-blocking function (in particular network-related, light-weight disk operation can be performed)
- threading (any thread should be launched prior to entering the real-time context)


Remote interaction with the controller(s)
==

Using MCControl RTC component
--

1. Implement a new function in the idl and its appropriate counter-part in `MCControllerService' that relies on `MCGlobalController` to call the appropriate function on the controller (as tedious as it sounds)
2. Implement `send_msg` or `send_recv_msg` depending on your need and perform the appropriate command parsing (e.g. `MCBCISelfInteract`)

Using mc_vrep
--

See mc_vrep for implementation details. mc_vrep already mimics the `send_msg` and `send_recv_msg` functionnality in the CLI.


FAQ
==

These are not the questions you're looking for.
