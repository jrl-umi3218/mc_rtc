# Changelog

## [Unreleased]

## [1.11.0] - 2022-07-08

### Highlights

`mc_rbdyn::Frame` and `mc_rbdyn::RobotFrame` have been introduced. This removes
the unnecessary distinction between bodies and surfaces that has existed in the
framework. (#247)

### API breaks

- `mc_rbdyn::Robots` cannot be created directly anymore, one has to go through
   a factory function (#247)
- `mc_rbdyn::Robots` stores indiviual robot as pointer and in particular
  `robots.robots()` returns a self reference instead of `vector<Robot>`,
  `Robots` behaves a little more like a vector now to mitigate the impact of this
  change (#247)
- `EndEffectorTask` and co. cannot use a `bodyPoint` in their constructor anymore, proper frames should be used instead (#247)
- `VectorOrientationTask` and co. cannot set or get the `bodyVector` after the fact anymore (#247)

### Deprecations

- Usage of `body` or `surface` in JSON/YAML is deprecated in favor of frame,
  the doc will only show `frame` but it should continue to work with all old
  JSON/YAML files (#247)
- Idem for specifications of relative targets (#247)

### Changes

- [mc_control] Many functionalities of FSM controllers -- notably simplified contact and collision manipulations and configuration based initialization -- are now available in basic controllers (#250)

### Added

- [mc_control] Controllers can specify extra frames in their configuration frame (#251)
- [mc_control] Controllers can specify different `init_pos` for different `MainRobot` (#251)
- [mc_rbdyn] Add an overload to load a robot module with a custom name
- [mc_rtc_utils] `mc_rtc::shared` is a wrapper around `std::enabled_shared_from_this` to allow
  reference-style API where the data is backed by a shared pointer (#247)
- [mc_tasks] All tasks (when it makes sense) can accept a frame at construction (#247)
- [mc_solver] `mc_solver::BoundedSpeedConstr` can be used to constraint a frame velocity (#247)
- [mc_solver] Active joints in a collision can be specified via `mc_rbdyn::Collision` objects (#262)
- [utils] `build_and_install` now supports Ubuntu Jammy (22.04) (#248)
- [utils] `build_and_install` can now install the Panda module (#249)

### Fixes

- [debian] Python bindings now include mc_observers
- [fsm::Meta] Fix missed iterations at the start of meta states (#257)
- [mc_control] Make robot position initialization consistant (#245)
- [mc_rbdyn] Fix initialization of the default body sensor (#258)
- [utils] Fix issues with pip/pip3 on Ubuntu Focal (#246)
- [utils] Honor forced selection of Python for mc_log_ui

## [1.10.0] - 2022-04-06

### Tutorials translation

mc_rtc website and tutorials are now available in [Japanese](https://jrl-umi3218.github.io/mc_rtc/jp/) (#225/#231)

### Removed

- mc_rbdyn_urdf is no longer used by mc_rtc, all related APIs that were deprecated have been removed in this release (#243)

### Added

- [mc_tasks/Stabilizer] Add CoM correction using bias estimation (#227)
- [mc_rtc] A source can be provided when adding GUI elements to simplify cleanup code (#237)
- [mc_solver] Added `FeedbackType::ClosedLoopIntegrateReal` (#240)
- [bindings] RPY utils are now available in mc_rbdyn bindings (#236)

### Changes

- [mc_control] Improve handling of metric grippers
- [mc_control] `mc_control::fsm::State::configure` now has a reasonable default implementation based on existing practice (#238)
- [mc_control] Data is now logged for all active robots in the controller automatically (#241)
- [mc_rtc] `log_error_and_throw` now defaults to `std::runtime_error` and includes a stacktrace (#224)

### Fixes

- [mc_control] `env/ground` is used as the new default environment module
- [mc_rbdyn] Normalize the rotation provided to `posW` for fixed base robots
- [mc_rbdyn] `RobotModule::init(rbd::parsers::ParserResult)` does not overwrite the reference joint order if it was already provided
- [mc_rbdyn] `RobotModule::expand_stance` does not include the `Root` joint
- [mc_tasks] Align posture task gains' GUI with other tasks
- [mc_tasks] Avoid cutoff period overwrite in impedance tasks (#227)
- [mc_tasks] Conserve `dimWeight` value when joints' selection is changed
- [ROS] Ensure all published quaternions are normalized (#242)

## [1.9.0] - 2022-02-10

### New build tool

- Introduce [mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild) a new tool to replaced the build_and_install script
  - See #221 for a list of benefits of mc-rtc-superbuild over build_and_install
  - build_and_install will be maintained and should keep working for a while but new users are advised to use the superbuild tool instead
  - You can start using mc-rtc-superbuild if you are using build_and_install already but this will require a full rebuild

### Changes

- hpp-spline has been dropped in favor of the actively maintained [ndcurves](https://github.com/loco-3d/ndcurves) (#210)
- Ubuntu Xenial (16.04) is no longer actively supported
- [mc_control] Allow plugin loaded at the controller level (#198)
  - See the global plugins tutorial for caveats and details regarding configuration
- [mc_control_fsm] A state final name is now correctly state when it enters `::configure` for the first time (#218)
- [mc_log_ui] Autoscaling has been improved (#222)
- [mc_rbdyn/mc_control_fsm] Contacts now compare equal regardless of the robots' order (i.e. `r1::s1/r2::s2 == r2::s2/r1::s1`) (#202)
- [mc_rtc] YAML improvments (#208/#213)
  - Allow to use the merge key from YAML 1.1
  - Disable y/n and yes/no bool conversion
- [mc_rtc] ObjectLoader sandboxing has been removed (#217)

### Fixes

- [build_and_install] Fix permissions issues on ninja logs
- [cmake] Handle building on aarch64 (#217)
- [cmake] Honor GNUInstallDirs (#217)
- [mc_control] Fix crash on controller restart/reset (#206)
- [mc_control] Fix issues with `removeRobot` (#207)
- [mc_log_ui] Fix many functionalities when running in Python 3 (#222)
- [mc_rbdyn] Fix stabilizer configuration loading (#206)
- [mc_solver] Automatically swap contact order if the first robot has no dof (#202)
- [mc_tasks] Correct name setting for `EndEffectorTask`

### Added

- [GUI] Forms can now have dynamic elements (#197)
- [GUI] New plots can be added to a plot after it has been added (#197)
- [mc_bin_utils/mc_log_ui] It is now possible to extract some keys from a log (#209)
- [mc_control] Added a multi-robot aware init method for `mc_control::MCGlobalController` (#206)
- [mc_control] Added a reset method for `mc_control::MCGlobalController` (#206)
- [mc_control] alphaOut (command velocity) and alphaDOut (command acceleration) are now logged by default (#222)
- [mc_control_fsm] API have been added to allow writing FSM tools (#222)
- [mc_rtc] Added `mc_rtc::ConfigurationFile` to simplify save/reload of configuration files (#222)
- [mc_tasks] MetaTask now has an `iterInSolver` method (#200)

## [1.8.2] - 2021-10-15

### New package

- Interface
  - [mc_mujoco](https://github.com/rohanpsingh/mc_mujoco/) for MuJoCo simulation

### Fixes

- [mc_control] Fix message ordering in the FSM (#193)
- [mc_rbdyn] Fix Robot::copy (#188)
- [mc_solver] Prevent a crash when doing contacts manipulation after a solver run (#196)
- [mc_tasks] Correctly set mimic joint target in PostureTask (#194)
- [Python] Allow Python controllers to be interrupted
- [utils] Fix a file descriptor leak that could occur in mc_log_ui

### Added

- [mc_rbdyn] Allow to iterate over all devices in a robot
- [mc_tasks] Add dimWeight support in the stabilizer configuration (#191)

## [1.8.1] - 2021-08-10

### Fixes

- Fix a unit test that prevent successfull packaging

## [1.8.0] - 2021-08-10

### New distribution channels

- Homebrew
  - mc_rtc is now available via Homebrew using the [mc-rtc/mc-rtc tap](https://github.com/mc-rtc/homebrew-mc-rtc/)
- vcpkg
  - mc_rtc is now available via vcpkg using the [mc-rtc/vcpkg-registry registry](https://github.com/mc-rtc/vcpkg-registry/), see also [How to start using registries with vcpkg](https://devblogs.microsoft.com/cppblog/how-to-start-using-registries-with-vcpkg/)

### Changes

- [GUI] Improve handling of default values in form (#164 #178)
- [mc_control] Change collision management for FSM controllers
  - Work-around Tasks issue when adding a collision where the first robot is non-actuated and the second is
  - Use the same manager for r1/r2 and r2/r1 collisions
  - Throw an exception if a wildcard collision does not create actual collisions
- [mc_solver] Always call `robot.forwardAcceleration()`
- [mc_tasks] The `MetaTask::name(const std::string &)` method is now virtual, this allows to rename sub-tasks as needed
- [plugins] Give finer control over plugin run behavior (#168)
- [utils] Use ninja to build projects when possible (only affects new builds)

### Added

- [global] Add `MC_RTC_DEVELOPER_MODE` option
- [GUI] Add the `Visual` element (#159)
- [Logging] Add support for `Eigen::Ref`
- [mc_rbdyn] Allow a `RobotModule` to specify extra collision objects from sch primitives (#163)
- [mc_robots] objects now have a dummy `FloatingBase` sensor
- [mc_solver] Add support for jerk bounds (#173)
- [mc_tasks] Add holding strategy for `mc_tasks::ImpedanceTask` (#143)
- [mc_tasks] PostureTask now supports `refVel` and `refAccel` (#165)
- [mc_tasks] PostureTask now supports `dimWeight` (#174)

### Fixes

- [Logger] Emit a warning if the logger queue is outrun in threaded mode; under normal circumstances this would indicate a hardware issue
- [mc_control] Correctly stop GUI sockets
- [mc_observers] Fix orientation transform issue between IMU and base link
- [mc_rbdyn] Avoid re-loading calibrator in `Robot::robotCopy`
- [mc_rbdyn] Save/Load material from JSON for visual elements
- [mc_rtc] Improve YAML -> JSON conversion
- [mc_tasks] Renaming the stabilizer task correctly renames its managed tasks (#171)
- [mc_tasks] Fix stabilizer disabling (#155)
- [ROS] Prevent crash on restart
- [utils] Fix string display in `mc_log_ui` (#180)
- [utils] Fix branch selection



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

[Unreleased]: https://github.com/jrl-umi3218/mc_rtc/compare/v1.11.0...HEAD
[1.11.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.11.0
[1.10.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.10.0
[1.9.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.9.0
[1.8.2]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.8.2
[1.8.1]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.8.1
[1.8.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.8.0
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
