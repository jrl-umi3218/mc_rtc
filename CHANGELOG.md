# Changelog

## [Unreleased]

### Added

- mc\_tasks::MomentumTask

### Fixes

- Enforce the list of supported robots provided by a controller

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

[Unreleased]: https://github.com/jrl-umi3218/mc_rtc/compare/v1.2.1...HEAD
[1.2.1]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.2.1
[1.2.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.2.0
[1.1.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.1.0
[1.0.1]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.0.1
[1.0.0]: https://github.com/jrl-umi3218/mc_rtc/releases/tag/v1.0.0

[stabilizer documentation]: https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/lipm-stabilizer.html
