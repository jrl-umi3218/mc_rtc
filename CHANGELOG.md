# Changelog

## [Unreleased]

## [1.7.0] - 2021-03-30

### New packages

- Interface
  - [mc_franka](https://github.com/jrl-umi3218/mc_franka) for libfranka integration
- Robot
  - [mc_panda](https://github.com/jrl-umi3218/mc_panda) for working with the Panda robot in mc_rtc
- GUI (experimental)
  - [mc_rtc-raylib](https://github.com/gergondet/mc_rtc-raylib/)
  - [mc_rtc-magnum](https://github.com/gergondet/mc_rtc-magnum/)

### Changes

- [Grippers] Allow to specify per-joint target angle and enforce joint limits in such cases (#125)
- [mc_log_ui] Greatly improve animation support (#126)
- [mc_rbdyn] Aliases are preserved as loading parameters for the RobotModule
- [mc_rtc] Allow to specify a data source for logging, entries can then be removed through this source (#115)
- [mc_rtc] When deserializing a Quaternion object, accept matrix and RPY representations
- [meta] Template projects for new controllers and robot modules are now available under the mc-rtc organization https://github.com/mc-rtc/
- [observers] Allow to specify optional observers and potentially failing observers (#123)
- [QHull] Update to QHull 2020.2 (#87)
- [Stabilizer] Add the possibility to estimate DCM bias (#105)
- [Stabilizer] Can now handle external forces (#95)
- [utils] Make Python executable modifiable, defaults on python3 for Ubuntu 20.04

### Added

- [global] Add MC_RTC_DISABLE_NETWORK option and MC_RTC_BUILD_STATIC option, allow to build a wasm version of the framework (#88)
- [GUI] `LineConfig` can be provided for `Polygon` elements (#118)
- [GUI] Add the `Robot` type to publish robots to non-ROS GUI
- [mc_rtc] Add support for logging `sva::ImpedanceVecd`
- [mc_tasks] Added ImpedanceTask (#100)
- [utils] Allow to specify a different (potentially absolute) build directory (#82)
- [utils] Auto-completion support for build-and-install (#83)
- [utils] Added support for HRP4CR (#106)

### Fixes

- [mc_rtc] Performance improvement for writing `sva::ForceVecd` and `sva::MotionVecd` in the log
- [ROS] Fix an issue where the publication rate was not enforced properly (#133)
- [Stabilizer] Fix min/max bug in contact computation (only affected logging/GUI) (#114)
- [Stabilizer] Fix `speed()` method (#124)
- [Stabilizer] Improve consistency when loading the configuration (#122)
- [utils] Correctly set DISABLE_ROS option in every case
- [web] Various typos in tutorials and website (#99, #107, #108, #116, #128, #132)
- [Windows] Fix pragma warnings and add `/bigobj` flag (#129)

## [1.6.0] - 2020-10-16

### Changes

- [cmake/loader] When mc_rtc is build in debug mode it will look for loadable libraries (controllers, robots, observers and plugins) in the debug sub-folder of the standard installation location (#78)
- [cmake/loader] On Windows, mc_rtc will now look for loadable libraries in the `bin/mc_[component]` folder instead of `lib/mc_[component]` and install helper libraries in the `bin` folder (#81)
- [FSM] `RemovePostureTask` is now `DisablePostureTask`, it disables either all FSM posture tasks for the state's duration if it is set to `true` or disable only the posture tasks for the provided robots (#80)
- [FSM] `RemovePostureTask` is kept for backward compatibility, it has the same capabilities as `DisablePostureTask` (#80)

### Added

- [FSM] Posture state has been introduced to interact with the global posture (#67)
- [FSM] HalfSitting can now handle any robot (#67)
- [mc_robots] Generate env aliases for the objects (#67)
- [mc_tasks] Add a simple constructor for StabilizerTask (#69)
- [cmake] Export mc_rtc observers as targets so they can be used as a base class
- [mc_rtc] Introduce mc_rtc::debug() (#78)
- [FSM] `constraints` can be used to load extra-constraints in any state (#80)
- [FSM] `tasks` can be used to load extra-tasks in any state (#80)
- [mc_trajectory] Introduce `SequenceInterpolator` to do linear interpolation between values over a time-sequence (#64)
- [mc_tasks] SplineTrajectoryTask can use `SequenceInterpolator` to use varying gains during the task execution (#64)

### Fixes

- [mc_observers] Fix configuration reading for anchor frame configuration (#69)
- Typo in CMake macros for controller build in catkin workspace (#75)
- Load libraries from a symlink (#76)
- [mc_log_ui] The open_log function returns a ditionnary as documented
- [mc_rtc] Configuration::empty() correctly returns false if the Configuration hold a value
- [mc_rtc] Configuration::load() does not (wrongly) load the full document under some circumstances

## [1.5.1] - 2020-09-14

### Added

- [mc_tasks] Allow to set the damping on PostureTask
- [mc_tasks] Log eval and speed in most tasks

### Fixes

- [mc_control] Add missing implementation for some (multi-robot) MCGlobalController functions

## [1.5.0] - 2020-09-09

### Changes

- Output torques are automatically computed when a dynamics constraint is added to the solver (#52)
- mc\_rtc::Configuration error reporting now reports the source of error (#60)
- Real robots instance are now created for all robots loaded by a controller
- Observer pipelines have been overhauled (#61); see the [refreshed tutorial](https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html) for details
- If a RobotModule provides torque derivative or joint acceleration bounds those will be enforced by (resp.) DynamicsConstraint and KinematicsConstraint

### Added

- [GUI] Added Table element with formatting support (#45/#47)
- [mc\_rbdyn] Add an API to find force sensors indirectly attached to a surface
- Support for simple collision shapes described in URDF (#53)
- Suppport for wildcard collision specifications (#53)
- GUI monitoring for collisions (#53)
- User configuration for the build and install script (#48)
- Support for specifying torque derivative and joint acceleration bounds in RobotModule
- [mc\_naoqi](https://github.com/jrl-umi3218/mc_naoqi) public release, an interface for SoftBank Robotics robots
- Support for logging `std::array<double, N>` (#62)
- Wrench completion criteria for SurfaceTransformTask (#63)
- Admittance sample controller (!215)

### Fixes

- Support for Ubuntu Focal/ROS noetic in utils script/CI
- Support for Debian Buster/ROS noetic in utils script/CI
- Fix several issues with special plots in mc\_log\_ui
- Publish up-to-date surfaces in ROS TF (#44)
- Improve documentation for completion criteria (#50)
- C-string overload for GenericLoader (fixes possibly ambiguous calls)
- Fix initialization issues
- Simplified device interface (#51)
- Fix an issue with collision transforms initialization
- Fix an issue with FSM executor perf entry logging
- Improved StabilizerTask/StabilizerState configurability (#23)

## [1.4.0] - 2020-06-11

### Changes

- [FSM] Deprecates the use of `robotIndex` in favor of robot name `robot`
- `bodyWrench` and `surfaceWrench` can now be computed with joints between the sensor and body/surface
- `Message` state can be used to display messages in the GUI as well as the CLI
- Benchmarks can now be compiled and run with the `build_and_install.sh` script
- Use RBDyn::Parsers instead of mc\_rbdyn\_urdf
- Functions that use mc\_rbdyn\_urdf were kept for backwards compatibility, they will be removed in a later release
- Improve error reporting when some tasks parameters are not set correctly
- `mc_rtc::log::info`, `mc_rtc::log::success`, `mc_rtc::log::warning` and `mc_rtc::log::error` replace the respective `LOG_*` macros, they use the spdlog library for asynchronous logging and modern formatting support
- `LOG_*` macros are now deprecated, they will be removed in a later release

### Added

- New benchmark for task allocation performance
- Support for `std::unordered_set` in `mc_rtc::Configuration`
- Added performance measurements to the FSM

### Fixes

- Fix and upgrade benchmarks compilation
- Correctly teardown the last FSM state
- Computed contact forces are logged with a unique identifier
- Improve Python 3 compatibility for mc\_log\_ui

## [1.3.0] - 2020-05-07

### Added

- mc\_tasks::MomentumTask
- mc\_rbdyn::RobotModule can specify a different URDF for showing the real robot
- mc\_log\_ui now has an icon and a dedicated .desktop file

### Changes

- mc\_solver::QPSolver always makes sure to udpate the problem size
- Many tasks throw as documented when pre-conditions are not met (e.g. surface does not exist or has no sensor attached)

### Changes

- Improve gripper handling so that `MCGlobalController` handles gripper update
- All robots' grippers are loaded and run by the controller
- Gripper interface is available from the `Robot` class
- Gripper logging and publishing are correctly handled in a way transparent to the controller
- Gripper joints are hidden from the posture task GUI
- Gripper safety settings can be provided at the Gripper definition level or at the RobotModule level

### Fixes

- Enforce the list of supported robots provided by a controller
- Throws an error if selected joints do not exist
- Correctly handle default case in Contact GUI menu

## [1.2.1] - 2020-03-18

### Added

- Friction support for FSM contacts

### Fixes

- Bug when removing contacts from the GUI
- Loading of mc\_rbdyn::Contact from YAML/JSON now supports friction

## [1.2.0] - 2020-03-17

### Added

- CoM task is now optional in `AddRemoveContact` state
- timeout API for ControllerClient
- XY and 3D plots support in mc\_log\_ui
- animation support in mc\_log\_ui
- mc\_rtc::DataStore to store arbitrary data and callbacks available across states
- Python bindings for robot().module() and controller configuration
- ZMP compensation control (ZMPCC) is now possible with the LIPMStabilizer
- Safety settings for gripper control can now be modified
- MetaTasksState can now deliver outputs relative to the completed task
- ParallelState output can be controlled more finely
- Contact friction can be set on a per-contact basis

### Changes

- Simplify usage of constraint helpers
- Create a default CompoundJointConstraint for the main robot
- mc\_log\_ui uses PyQt5 instead of PySide

### Fixes

- `mc_tasks::GazeTask`: fix a possible division by zero
- ROS support on macOS in build script
- Fix ROS linking issue, ensuring the ROS based tasks are available if the plugin is loaded
- Check 3rd-party CMake targets existence before creating them
- Improve constraint helpers documentation
- Fix typos in documentation and website
- Pass down mc\_rtc version to Python bindings
- Improve build script ability to handle incremental updates
- Correct issue in calling MCController::loadRobot after the controller was started
- CoMTask logging correctly reports the current CoM
- Changing active/inactive joints while a task is in the solver will actually do nothing (as documented)

## [1.1.0] - 2020-01-15

### Added

- `mc_tasks::lipm_stabilizer::StabilizerTask` and
  `mc_control::fsm::StabilizerStandingState` allow to use a state-of-the-art
  biped stabilizer in a stand-alone controller or as a parallel state in an FSM.
  Refer to the [stabilizer documentation] for more details
- JSON/YAML documentation for the FSM states provided by the framework

### Changes

- Split interface and macros header to improve incremental compilations of the
  framework. In some cases it might require to include the macros explicitely
- `virtual void update()` in the `mc_tasks::MetaTask` interface has been
  changed to `virtual void update(mc_solver::QPSolver & solver)`

### Fixes

- JVRC1 and env modules now correctly set `_visual`

## [1.0.1] - 2020-01-02

### Added

- `mc_rtc::MC_RTC_VERSION` and `mc_rtc::version()` for compile/run-time discrepancies checking

### Fixes

- [mc\_rbdyn] Allow aliases to specify a string
- [mc\_rbdyn] Allow loading aliases of aliases

## [1.0.0] - 2019-12-23

Initial release

[Unreleased]: https://github.com/jrl-umi3218/mc_rtc/compare/v1.7.0...HEAD
[1.7.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.7.0
[1.6.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.6.0
[1.5.1]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.5.1
[1.5.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.5.0
[1.4.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.4.0
[1.3.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.3.0
[1.2.1]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.2.1
[1.2.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.2.0
[1.1.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.1.0
[1.0.1]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.0.1
[1.0.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.0.0

[stabilizer documentation]: https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/lipm-stabilizer.html
