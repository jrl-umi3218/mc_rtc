---
layout: tutorials
toc: true
---

In this page you will find a list of all available sample controllers provided with the framework, along with a brief description of how to run them and what to expect. More complex controllers will be further detailed in their own tutorial page.

# Posture

The `Posture` controller (see [online demonstration](https://mc-rtc-demo.netlify.app/#robot=JVRC1&controller=EndEffector)) is the simplest controller in the framework. It adds the following tasks and constraints to the QP solver.



**Tasks**
- {% doxygen mc_tasks::PostureTask %}:
Make each of the robot's DoF target a given joint position. By default this controller will target the current robot encoder values when available, or the default stance for that robot otherwise ({% doxygen mc_rbdyn::RobotModule::stance() %}). Joints can be moved through the joint sliders in the GUI (`Tasks -> posture_jvrc1 -> Target`).

**Constraints**
- {% doxygen mc_solver::ContactConstraint %}: A controller must always have a contact constraint, but it can have an empty set of contacts.
- {% doxygen mc_solver::KinematicsConstraint %}: Joint limits constraint.
- {% doxygen mc_solver::CollisionsConstraint %}: Self-collision avoidance constraint.
- {% doxygen mc_solver::CompoundJointConstraint %}: Handle joint limits for compound joints (joints whose limits depend on the value of another joint).

**Supported robots**
All [robots]({{site.baseurl}}/robots.html) supported by the framework. Note that for custom robots, all you need to do is define a default set of collision pairs and limits for the `CompoundJointConstraint` if appropriate (see the tutorial on [integrating a new robot]({{site.baseurl}}/tutorials/advanced/new-robot.html)).

**Running**
To run this controller, simply put in [your mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html), and [run the controller]({{site.baseurl}}/tutorials/introduction/running-a-controller.html) using your favorite interface.

```yaml
MainRobot: JVRC1
Enabled: Posture
```

# CoM

The `CoM` sample controller (see [online demonstration](https://mc-rtc-demo.netlify.app/#robot=JVRC1&controller=CoM)) has similar tasks and constraints as the `Posture` controller above.

**Tasks**
- {% doxygen mc_tasks::PostureTask %}: A posture task with low-weight to provide a default target for all joints.
- {% doxygen mc_tasks::CoMTask %}: Task controlling the robot's CoM. By default this controller targets the current task position. You can move this target using the interactive markers in the GUI (rviz).

**Constraints**
- {% doxygen mc_solver::ContactConstraint %}: Adds contacts between the left/right foot and the ground environment.
- {% doxygen mc_solver::DynamicsConstraint %}: In addition to the joint limits constraint provided by the {% doxygen mc_solver::KinematicsConstraint %} this constraint enables computation of the joint torques and friction cones.
- {% doxygen mc_solver::CollisionsConstraint %}: Self-collision avoidance constraint.
- {% doxygen mc_solver::CompoundJointConstraint %}: Handle joint limits for compound joints (joints whose limits depend on the value of another joint).

**Supported robots**
All biped [robots]({{site.baseurl}}/robots.html) supported by the framework. Note that for custom robots, all you need to do is define a `LeftFoot` and `RightFoot` surface in [your robot description]({{site.baseurl}}/tutorials/advanced/new-robot.html).


**Running**
To run this controller, simply put in [your mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html), and [run the controller]({{site.baseurl}}/tutorials/introduction/running-a-controller.html) using your favorite interface.

```yaml
MainRobot: JVRC1
Enabled: CoM
```

# EndEffector

The `EndEffector` controller (see [online demonstration](https://mc-rtc-demo.netlify.app/#robot=JVRC1&controller=EndEffector)) has the same tasks and constraints as the `CoM` controller above. In addition to controlling the CoM position, it adds an {% doxygen mc_tasks::EndEffectorTask %} to control the position and orientation of the robot's hand end-effector.

**Supported robots**
All biped [robots]({{site.baseurl}}/robots.html) supported by the framework. Note that for custom robots, all you need to do is define a `LeftFoot` and `RightFoot` surfaces and an `r_wrist` body in [your robot]({{site.baseurl}}/tutorials/advanced/new-robot.html).

**Running**
To run this controller, simply put in [your mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html), and [run the controller]({{site.baseurl}}/tutorials/introduction/running-a-controller.html) using your favorite interface.

```yaml
MainRobot: JVRC1
Enabled: EndEffector
```

# Text

The `Text` sample controller demonstrate the use of {% doxygen mc_tasks::MetaTaskLoader %} and {% doxygen mc_solver::ConstraintSetLoader %} to respectively load [tasks]({{site.baseurl}}/json.html#MetaTask) and [constraints]({{site.baseurl}}/json.html#ConstraintSet) from their YAML configuration.

**Running**
To run this controller, simply put in [your mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html), and [run the controller]({{site.baseurl}}/tutorials/introduction/running-a-controller.html) using your favorite interface.

```yaml
MainRobot: JVRC1
Enabled: Text
```

The default configuration for this controller is similar to the `EndEffector` controller. You can try writing your own configuration in `~/.config/mc_rtc/controllers/Text.yaml`. For example, the following configuration re-creates the `CoM` controller and moves the `CoM` above the `LeftFoot` surface:

```yaml
Text:
  constraints:
  - type: dynamics
  - type: contact
  - type: collision
  - type: compoundJoint
  tasks:
  - type: com
    weight: 1000
    above: [LeftFoot]
  contacts:
  - r1Surface: LeftFoot
    r2Surface: AllGround
    isFixed: false
  - r1Surface: RightFoot
    r2Surface: AllGround
    isFixed: false
```

# Admittance

The `Admittance` sample controller demonstrates the use of the {% doxygen mc_tasks::force::AdmittanceTask %} though a simple FSM making the `JVRC1` robot push a wall with a desired force.

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/B_L_xPynhvU" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>


**Detailed description**: see the [Admittance Sample Controller tutorial]({{site.baseurl}}/tutorials/samples/sample-admittance.html)

**Supported robots**: `JVRC1`.

**Prerequisites**: Please make sure that the hand force sensors have been calibrated beforehand using the [ForceSensorCalibration controller](https://github.com/jrl-umi3218/mc_force_sensor_calibration_controller).

**Running**

To run this controller, simply put in [your mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html), and [run the controller]({{site.baseurl}}/tutorials/introduction/running-a-controller.html) using your favorite dynamic simulation (e.g Choreonoid...).

**See also**

For a similar but more advanced example, see the [ExternalForces](#externalforces) sample.

```yaml
MainRobot: JVRC1
Enabled: AdmittanceSample
```

# Impedance

The `Impedance` sample controller demonstrates the use of the {% doxygen mc_tasks::force::ImpedanceTask %} to simulataneously perform position and force control of the robot's end-effector.

**Supported robots**: `JVRC1Fixed`.

**Prerequisites**: Please make sure that the hand force sensors have been calibrated beforehand using the [ForceSensorCalibration controller](https://github.com/jrl-umi3218/mc_force_sensor_calibration_controller).

**Running**

To run this controller, simply put in [your mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html), and [run the controller]({{site.baseurl}}/tutorials/introduction/running-a-controller.html) using your favorite dynamic simulation (e.g Choreonoid...). In choreonoid you can apply external forces by clicking on the moving hand and dragging it. It will follow you to some extent and return to it's expected position along the trajectory when you relase the force. Note that you may add additional {% doxygen mc_tasks::force::ImpedanceTask %} to the free end-effectors throught the GUI (`Global  -> Add Tasks -> ImpedanceTask`).

```yaml
MainRobot: JVRC1Fixed
Enabled: Impedance
```

# LIPMStabilizer

The `LIPMStabilizer` sample controller demonstrates the use of the {% doxygen mc_tasks::StabilizerTask %} and its corresponding convenience FSM state {% doxygen mc_state::StabilizerStandingState %} though a simple quasi-static FSM making the robot stand and step while keeping balance.

**Detailed description**: see [the LIPM Stabilizer tutorial]({{site.baseurl}}/tutorials/recipes/lipm-stabilizer.html)

**Supported robots**: All biped [robots]({{site.baseurl}}/robots.html) supported by the framework.

**Running**

To run this controller, simply put in [your mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html), and [run the controller]({{site.baseurl}}/tutorials/introduction/running-a-controller.html) using your favorite dynamic simulation (e.g Choreonoid...).

```yaml
MainRobot: JVRC1
Enabled: LIPMStabilizer
```

# ExternalForces

The `ExternalForces` sample controller demonstrates a biped robot exerting specified external forces with {% doxygen mc_tasks::lipm_stabilizer::StabilizerTask %} and {% doxygen mc_tasks::force::ImpedanceTask %} though a simple FSM making the `JVRC1` robot push a wall with desired forces.

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/in3cUozkU-A" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

**Supported robots**: `JVRC1`.

**Prerequisites**: Please make sure that the hand force sensors have been calibrated beforehand using the [ForceSensorCalibration controller](https://github.com/jrl-umi3218/mc_force_sensor_calibration_controller).

**Running**

To run the sample, you need a dynamic simulator in order to simulate the force sensors. This tutorial is intended to be used with {% link mc_openrtm %} and {% link Choreonoid %}, and the provided simulation file `sim_mc_wall.cnoid`. If you use another simulator, you will need to adapt the instructions, and create a scene with a wall `55cm` away from the robot.

Put in [your mc_rtc configuration]({{site.baseurl}}/tutorials/introduction/configuration.html), and [run the controller]({{site.baseurl}}/tutorials/introduction/running-a-controller.html) with Choreonoid. The robot reaches both hands to the wall and applies the specified external forces of the sine wave to the wall with leaning forward.

```yaml
MainRobot: JVRC1
Enabled: ExternalForces
```
