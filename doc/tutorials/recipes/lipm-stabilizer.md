---
layout: tutorials
toc: true
---

## A bit of history

The stabilizer presented in this tutorial was originally implemented as part of Dr. Stéphane Caron's [LIPM Walking Controller](https://github.com/stephane-caron/lipm_walking_controller). This implementation has been intensively used for walking, including stair climbing and extensive locomotion, notably as part of an Airbus' manufacturing use-case performed in-situ at their industrial factory. The implementation proposed in `mc_tasks::lipm_stabilizer::StabilizerTask` is an integration of the method proposed in Dr. Caron's original implementation. No significant conceptual alteration to the stabilization algorithm were made. Walking is currently provided in a separate  [controller](https://github.com/jrl-umi3218/lipm_walking_controller).

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/vFCFKAunsYM" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>


## Overview of the stabilizer

For details on the stabilization algorithm, please refer to the original publication by Stéphane Caron, Abderrahmane Kheddar and Olivier Tempier: [Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control](https://scaron.info/publications/icra-2019.html) published at *ICRA 2019, Montreal, Canada, May 2019.*

This stabilizer is implemented here in `mc_tasks::lipm_stabilizer::StabilizerTask` as a `MetaTask`, i.e a task that manages the following additional tasks:

- A `CoMTask` to track the desired CoM position computed by the stabilizer
- Two `CoPTask` to track desired wrench under each of the robot's feet
- Two `OrientationTasks` on respectively the waist and torso bodies, used for regularization of the upper-body behaviour

The `StabilizerTask` role is to compute targets to provide to these tasks such that the real robot system tracks as-best-as-possible the behaviour of a reference linear inverted pendulum model. As such, to stabilize dynamic trajectories, the user must provide the following inputs to the stabilizer:

- Position of the desired `CoM`
- Velocity of the desired `CoM`
- Acceleration of the desired `CoM`
- Position of the desired `ZMP`

This full reference state can be provided by:

```cpp
void StabilizerTask::target(
            const Eigen::Vector3d & com,
            const Eigen::Vector3d & comd,
            const Eigen::Vector3d & comdd,
            const Eigen::Vector3d & zmp);
```

For convenience, a helper function is provided to give appropriate pendulum targets for static position. It is important to note that this helper is only suitable to make the stabilizer keep the robot in a given configuration, but should not be used for dynamically moving the CoM.

```cpp
void StabilizerTask::staticTarget(const Eigen::Vector3d & com, double zmpHeight = 0);
```

It is your responsibility to provide a valid reference state. In case of walking, this is typically computed by a Model Predictive Controller (MPC) such that dynamic stability is ensured in-between footsteps.

## State observation

The aim of the stabilizer is to make the real system track the desired reference pendulum. As such, it requires the observation of the following real robot state:

- Position of the `CoM`
- Velocity of the `CoM`
- Contact wrenches

You must setup a suitable [observer pipeline](observers.html) to estimate the required quantities. The following pipeline can be used, it is equivalent to the original implementation:

```yaml
# Observes real robot state
RunObservers: [Encoder, KinematicInertial]
# Updates the Controller::realRobot() instance from the observed state
UpdateObservers: [Encoder, KinematicInertial]
```

- The `Encoder` observer computes the body position and velocities based on encoder measurements
- The `KinematicInertial` observer computes the floating base's pose and its velocity based on IMU measurement and a reference anchor frame provided in-between the robot's feet.

**Important note:** you are expected to provide a valid continuous anchor frame. For simplicity, the stabilizer implementation provides it as

```cpp
sva::PTransformd anchorFrame() const;
sva::PTransformd anchorFrameReal() const;
```

For now these values must be manually passed at each iteration to the `KinematicInertial` observer by calling:

```cpp
mc_controller::Controller & ctl = ...; // Controller's instance
mc_tasks::lipm_stabilizer::StabilizerTask & stabilizer = ...; // Stabilizer instance
ctl.anchorFrame(stabilizer.anchorFrame());
ctl.anchorFrameReal(stabilizer.anchorFrameReal());
```

With this, the observer pipeline will automatically update the `realRobot()` instance with the estimated state, which the stabilizer task will use to compute the tasks' targets.

## Configuring the gains

The following parameters determines how the stabilizer reacts to observed perturbation of the desired pendulum state.
See [Stephane's documentaion](https://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/stabilizer.html) to learn more about how to tune those gains.

```yaml
####
# Sample stabilizer configuration for the JVRC robot
# The most important entries are the
# - dcm_tracking: controls how the stabilizer reacts to perturbation of the DCM
# - admittance: controls the contact admittance
###
# Sole-floor friction coefficient
friction: 0.7
# Configuration of the tasks managed by the stabilizer
tasks:
  com:
    stiffness: [1000, 1000, 100]
    weight: 1000
    active_joints: [Root,
                    R_HIP_Y, R_HIP_R, R_HIP_P, R_KNEE, R_ANKLE_P, R_ANKLE_R,
                    L_HIP_Y, L_HIP_R, L_HIP_P, L_KNEE, L_ANKLE_P, L_ANKLE_R]
    height: 0.85
  contact:
    damping: 300
    stiffness: 1
    weight: 10000
  pelvis:
    stiffness: 10
    weight: 100
  torso:
    stiffness: 10
    weight: 100
    pitch: 0
fdqp_weights:
  net_wrench: 10000
  ankle_torque: 100
  pressure: 1
vdc:
  frequency: 1
  stiffness: 1000
admittance:
  cop: [0.01, 0.01]
  dfz: 0.0001
  dfz_damping: 0
dcm_tracking:
  gains:
    prop: 5.0
    integral: 10
    deriv: 0.5
  derivator_time_constant: 1
  integrator_time_constant: 10

# If you are using the same configuration file with different robots, you can provide per-robot configuration as well
jvrc1:
  admittance:
    cop: [0.02, 0.02]

  ```

Looks daunting? Don't worry, you most likely won't need to provide those gains explicitely, unless you are tuning the stabilizer for a new robot. A default stabilizer configuration must be provided by the `RobotModule` and can be accessed within your controller by

```cpp
mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
```

You can easily load the task from configuration:

```cpp
auto stabilizerTask = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(ctl.solver(), config);
ctl.solver().addTask(stabilizerTask);
```

When doing so, the stabilizer is configured in the following way:

- Default configuration from the `RobotModule::defaultLIPMStabilizerConfiguration()`. A valid configuration is provided for all [robots]({{site.baseurl}}/robots.html) supported by `mc_rtc`.
- YAML configuration for the task
- Per-robot YAML configuration can be used to modify any of these values for a specific robot.

See the [JSON schema](../../json.html#MetaTask/LIPMStabilizerTask) for details on the YAML configuration format.

## Using the Stabilizer (Manually)

To create a stabilizer task within your own controller, you simply need to instanciate the `mc_tasks::lipm_stabilizer::StabilizerTask`, and

```cpp
// Load default configuration from robot module
auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
// Create the stabilizer task
auto t = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
          solver().robots(),
          solver().realRobots(),
          robotIndex,
          stabiConf.leftFootSurface,
          stabiConf.rightFootSurface,
          stabiConf.torsoBodyName,
          solver().dt());
// Reset the task targets and default configuration
t->reset();
// Apply stabilizer configuration (optional, if not provided the default configuration from the RobotModule will be used)
t->configure(stabiConf);
// Set contacts (optional, the stabilizer will be configured in double support using the current foot pose as target for each contact by default)
t->setContacts({ContactState::Left, ContactState::Right});
```

Additionally, you may load additional configuration settings and targets from an `mc_rtc::Configuration` object:

```cpp
// Load default configuration from robot module
auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
// mc_rtc::Configuration object containing valid stabilizer configuration (see JSON schema documentation)
auto conf = ...
// Optional: Load additional configuration from an mc_rtc::Configuration object
stabiConf.load(config);
// Create the stabilizer task
auto t = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
          solver().robots(),
          solver().realRobots(),
          robotIndex,
          stabiConf.leftFootSurface,
          stabiConf.rightFootSurface,
          stabiConf.torsoBodyName,
          solver().dt());
// Reset the task
t->reset();
// Apply stabilizer configuration
t->configure(stabiConf);
// Load additional properties from configuration (contact targets, com target, etc)
t->load(solver, config);
```

By default, if no contacts and targets are specified, the task will start in double support and attempt to maintain the current feet pose and CoM position of the control robot. You may give targets to the stabilizer using `StabilizerTask::target(com, comd, comdd, zmp)` (see above).

## Using the Stabilizer (FSM)

The previous sections have introduced generalities about the stabilizer, how to create it and configure its gains. Using it is now as simple as selecting the appropriate contact mode (left support, right support, double support), and providing a valid reference state. Additionally, for the most common use-cases, `FSM` facilities are provided to help with its use.

A default FSM state `StabilizerStandingState` is provided. This state automatically creates a stabilizer task from configuration, and provides simple targets to the `CoM` and `Contacts`. The trajectory of the CoM is computed according to a simple spring-damper, and dynamic pendulum references are computed using a LIPM pendulum model. This reference is given to the stabilizer at every timestep.

You can configure the state in `YAML` as follows

```yaml
##
# This state keeps the robot standing at it's current position in double support
##
Stabilizer::Standing:
  base: StabilizerStandingState
  # This stiffness controls the spring-damper computation of the reference CoM position
  # damping is automatically computed as 2*sqrt(stiffness)
  stiffness: 5
  # StabilizerTask configuration (see previous section)
  StabilizerConfig:
    type: lipm_stabilizer
    leftFootSurface: LeftFootCenter
    rightFootSurface: RightFootCenter
    enabled: true
    contacts: [Left, Right]
```

Using this state, it is very easy to move the CoM (or keep it at a desired position)

```yaml
##
# Make the CoM move to a point centered above both of the robot's feet contact
#
# Completion:
# - OK when the dcm reaches the provided threshold
##
Stabilizer::GoCenter:
  base: Stabilizer::Standing
  above: Center
  completion:
    dcmEval: [0.005, 0.005, 0.05]

##
# Make the CoM move to a point above the left foot ankle
##
Stabilizer::GoLeft:
  base: Stabilizer::GoCenter
  above: LeftAnkle

##
# Make the CoM move to a point above the right foot ankle
##
Stabilizer::GoRight:
  base: Stabilizer::GoLeft
  above: RightAnkle
```


A simple library of predefined states for the most common actions (including the ones above) is provided in `mc_control/fsm/states/data/StabilizerStanding.yaml`. These states are intended to be put in parallel to other states to achieve more complex actions, such as the ones showcased in the next section.


## LIPMStabilizer sample FSM

To get started with the stabilizer, the user may refer to the provided sample controller `LIPMStabilizer`. To test it, simply add the following to your `mc_rtc.yaml` configuration file:

```yaml
Enabled: LIPMStabilizer
```

And run an interface supporting dynamic simulation (such as `mc_vrep` or `choreonoid`). This default FSM lets you move the CoM from left to right foot, and provides the following FSMs:

- `Stabilizer::Standing` : keeps the CoM at its current observed position. While in this state, you can freely move the CoM from `Left` to `Right` ankle in the GUI, or any other position. You may go to the "Tasks->Stabilizer" tab to see and modify the stabilizer gains. Additionally the "Debug" tab lets you add live plots visualizing the stabilizer behaviour.
- `AlternateFeetLifting` : alternatively lift the left and right foot. This is essentially a quasi-static motion, where the `StabilizerStandingState` is used to move the CoM above the left (resp right) ankle, configure the stabilizer in single support, then use it in parallel to tasks on the right (resp. left) swing foot (`SurfaceTransform` for the liftoff phase, `AdmittanceTask` for putting the foot down).
- `AlternateFeetLiftingManual`: same as above, but transitions are triggered by the user. Useful to tune the stabilizer's behaviour
- `StepForward`: performs one `20cm` step foward using a quasi-static motion. If using on a real robot, be careful as the swing foot trajectory is merly a spline with no early/late impact handling.
- `StepBackward`: performs one `10cm` step backward using a quasi-static motion.

## Interacting with the stabilizer from another FSM states

The stabilizer states provides a number of callbacks for other states to configure it through the user of the [datastore](datastore.html). The following table lists the available callbacks.

<table class="table">
  <thead>
    <tr>
      <th scope="col">Name</th>
      <th scope="col">Type</th>
      <th scope="col">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>StabilizerStandingState::getCoMTarget</td>
      <td><pre>const Eigen::Vector3d & ()</pre></td>
      <td>Returns the CoM target used by the stabilizer</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setCoMTarget</td>
      <td><pre>void (const Eigen::Vector3d &)</pre></td>
      <td>Modifies the CoM target used by the stabilizer</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::getStiffness</td>
      <td><pre>double ()</pre></td>
      <td>Returns the CoM tracking stiffness</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setStiffness</td>
      <td><pre>void (double)</pre></td>
      <td>Modifies the CoM tracking stiffness<br/>Also sets damping to 2*sqrt(stiffness)</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::getDamping</td>
      <td><pre>double ()</pre></td>
      <td>Returns the CoM tracking damping</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setDamping</td>
      <td><pre>void (double)</pre></td>
      <td>Modifies the CoM tracking damping</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setCoMWeight</td>
      <td><pre>void (double)</pre></td>
      <td>Modifies the CoM task weight</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setCoMStiffness</td>
      <td><pre>void (const Eigen::Vector3d &)</pre></td>
      <td>Modifies the CoM task stiffness</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setPelvisWeight</td>
      <td><pre>void (double)</pre></td>
      <td>Modifies the Pelvis task weight</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setPelvisStiffness</td>
      <td><pre>void (double)</pre></td>
      <td>Modifies the Pelvis task stiffness</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setTorsoWeight</td>
      <td><pre>void (double)</pre></td>
      <td>Modifies the Torso task weight</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setTorsoStiffness</td>
      <td><pre>void (double)</pre></td>
      <td>Modifies the Torso task stiffness</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::getConfiguration</td>
      <td><pre>lipm_stabilizer::StabilizerConfiguration ()</pre></td>
      <td>Returns the stabilizer configuration</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::setConfiguration</td>
      <td><pre>void (const lipm_stabilizer::StabilizerConfiguration &)</pre></td>
      <td>Sets the stabilizer configuration</td>
    </tr>
  </tbody>
</table>

Example:

```cpp
// Adds a GUI element to the controller instance ctl, providing the ability to read and modify the stiffness of the stabilizer
// This may for instance be used from another FSM state running in parallel with the StabilizerStandingState.
ctl.gui()->addElement({"DatastoreExample"},
    mc_rtc::gui::NumberInput("Stiffness",
      [&ctl]()
      {
        return ctl.datastore().call<double>("StabilizerStandingState::getStiffness");
      },
      [&ctl](double K)
      {
        ctl.datastore().call("StabilizerStandingState::setStiffness", K);
      }));
```
