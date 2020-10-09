---
layout: tutorials
toc: true
---

This tutorial explains usage and concepts of the `AdmittanceSample` sample controller provided with the framework. This sample demonstrates the use of the {% doxygen mc_tasks::force::AdmittanceTask %} to regulate forces on `JVRC1` hand while pushing against a wall.

This controller follows the following steps:

1. Trajectory to place the hand in front of the wall
2. Use of `AdmittanceTask` to establish contact
3. Regulation of the normal force to `20N`
4. Updating the wall position according to kinematics
4. Decreasing the force to prepare for releasing the hand
5. Moving hand back
6. Going back to halfsitting posture (with hand-wall collision avoidance)

## Running the controller

Sources for this sample are located in [src/mc_control/samples/Admittance](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Admittance) with the following main files:
- [etc/AdmittanceSample.in.yaml](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Admittance/etc/AdmittanceSample.in.yaml): FSM YAML Configuration (see this [tutorial]({{site.baseurl}}/tutorials/recipes/fsm.html))
- [src/states/UpdateWall.h](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Admittance/src/states/UpcdateWall.h), [src/states/UpdateWall.cpp](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Admittance/src/states/UpcdateWall.cpp): C++ state that updates the wall position according to the robot's gripper position

To run the sample, you need a dynamic simulator in order to simulate the force sensors. This tutorial is intended to be used with {% link mc_openrtm %} and {% link Choreonoid %}, and the provided simulation file `sim_mc_wall.cnoid`. If you use another simulator, you will need to adapt the instructions, and create a scene with a wall `55cm` away from the robot.

First, you need the following `~/.config/mc_rtc/mc_rtc.yaml` file:

```yaml
MainRobot: JVRC1
Enabled: AdmittanceSample
Timestep: 0.005
```

Then to run the sample, use:

```sh
$ (roscore &) # Ensure you have a roscore running (for rviz visualization)
$ cd /usr/local/share/hrpsys/samples/JVRC1
$ choreonoid sim_mc_wall.cnoid
$ roslaunch mc_rtc_ticker controler_display.launch
```

Then start the simulation by clicking on the green arrow in Choreonoid's interface. You should see the following output:

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/B_L_xPynhvU" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>


The following graph depicts the results of the force tracking along the gripper's `z` axis. Note the spike at `t=6s` corresponding to an impact between the gripper and the wall. About `1s` later, the force target of `-20N` is realized, and tracked for a few seconds. Then the pressure is released.

<img src="img/admittance.svg" alt="admittance results" class="img-fluid" />

## Explanations

Most of this sample is achieved by describing the states and task's objective in the FSM configuration `etc/AdmittanceSample.in.yaml`. First, a transition map describes how the transitions between state occur.

```yaml
transitions:
- [RightHandToWall,            OK, RightHandPushAdmittance, Auto]
- [RightHandPushAdmittance,    OK, RightHandKeepAdmittance, Auto]
- [RightHandKeepAdmittance,    OK, UpdateWallFromKinematics, Auto]
- [UpdateWallFromKinematics,   OK, RightHandReleaseAdmittance, Auto]
- [RightHandReleaseAdmittance, OK, RightHandMoveBack, Auto]
- [RightHandMoveBack,          OK, StandingHalfSitting, Auto]

# Initial state
init: RightHandToWall 
```

You should have easily recognized the main steps of the sample just by reading this transition map. Now let's look at what the main states are doing.

### StandingBase: Managing the CoM and Chest

During the whole experiment, we want the robot's CoM to remain centered between the feet. This is achieved by using a {% doxygen mc_tasks::CoMTask %}. Additionally, we add an {% doxygen mc_tasks::OrientationTask %} on the robot's chest to prevent it from moving too much and provide a more human-like motion. This state is used as the base state for all other states in the FSM, thus every state will inherit this behaviour.

```yaml
  # Keep the CoM centered between the feet, and the chest upright
  StandingBase:
    base: MetaTasks
    tasks:
      CoM:
        type: com
        above: [LeftFoot, RightFoot]
        weight: 2000
        stiffness: 5
      KeepChest:
        type: orientation
        body: WAIST_R_S
        weight: 100
        stiffness: 1
```

### RightHandToWall: Hand trajectory 

Next, we want to move the hand to a location in front of the robot near the wall. To do so, we use the {% doxygen mc_tasks::BSplineTrajectoryTask %}, that provides a bspline trajectory parametrized in terms of waypoints in both position and orientation. The task controls a desired robot surface to follow this trajectory.

```yaml
  # Trajectory to bring hand close to the wall
  RightHandToWall:
    base: StandingBase
    tasks:
      # inherits CoM and KeepChest tasks from StandingBase state
      # and adds a bspline_trajectory task
      RightHandTrajectory:
        type: bspline_trajectory
        surface: RightGripper
        weight: 1000
        stiffness: 50
        duration: 4
        dimWeight: [1,1,1, 1, 0.5, 0.5]
        displaySamples: 100
        target:
          translation: [0.45, -0.4, 1.1]
          rotation: [1.57, 0, 1.57]
        controlPoints: [[0.17, -0.5, 0.85]]
        completion:
          timeElapsed: true
```

### Admittance: force control

Next, the `RightHandPushAdmittance` state uses the {% doxygen mc_tasks::AdmittanceTask %} to regulate the normal force on the hand and establish contact with the wall. The admittance task works by computing a desired end-effector velocity based on the error between the measured wrench (from force-torque sensor) and the desired wrench. The `admittance` coefficient regulates how fast the end-effector moves depending on the magnitude of the error along that axis. Note that in order for the task to track velocity, you want to configure it to have low-stiffness (position control) and high-damping (velocity control) along the directions where you wish to track forces. Also note that all wrenches are expressed in surface frame (here the `RightGripper` surface frame).

```yaml
  # Increase applied force until we reach 20N
  RightHandPushAdmittance:
    base: StandingBase
    tasks:
      RightHandAdmittance:
        type: admittance
        surface: RightGripper 
        # Tracks the forces along the normal axis of the gripper surface with an admittance coefficient of 0.001
        # Note the high-damping and low stiffness on that axis.
        # All other axis are position controlled and will keep the current gripper position as much as possible 
        admittance: [0,0,0,0,0,0.001]
        stiffness: [10, 10, 10, 10, 10, 1]
        damping: [6.3, 6.3, 6.3, 6.3, 6.3, 300]
        maxVel:
          linear: [0.2, 0.2, 0.5]
          angular: [0.2, 0.2, 0.2]
        # Target a desired force of 20N along the Z axis of the gripper surface
        wrench:
         force: [0, 0, -20]
         couple: [0, 0, 0]
        # Rotate the gripper such that the fingers point towards the wall  
        targetRotation: [1.57, 0, 1.57]
        # The task will complete one a force of -20N has been reached. All other axes are ignored.
        completion:
          wrench:
            force: [.nan, .nan, -20]
            couple: [.nan, .nan, .nan]
```

The next state `RightHandKeepAdmittance` uses the ParallelState ({% doxygen mc_control::fsm::ParallelState %}) to keep the previous state running for a given duration. While the state is running, it'll attempt to keep the applied force constant. Thus if you push the robot forward, the hand will move backwards to reduce the applied force, if you pull the robot, the opposite will occur.

### Update wall position from kinematics

The admittance task described above will cause the gripper to move towards the wall until the desired force has been reached. In our controller, we assumed the wall position to be at:

```yaml
robots:
  wall:
    module: env/ground # reuse the ground model, but rotate it vertically to simulate a wall
    init_pos:
      # The wall position is put 5cm in front of choreonoid's one
      translation: [0.50, 0.0, 0.0]
      rotation: [0.0, -1.57, 0.0]
```

However, this expected position does not necessarily match the actual wall position: the robot might be starting from a slightly different configuration than expected. However, we know, thanks to the previous admittance states that the robot's gripper has reached the wall. This information can be exploited to update our model's wall position accordingly by exploiting the robot kinematics (and assuming non-slippage of the contacts). This is the role of the `UpdateWall` state implemented in C++. It reads the current estimated position of the robot's fingertips, and updates the wall position such that the fingertips are in contact with it.

Note that this state uses the robot state observed by the {% doxygen mc_observers::KinematicInertialObserver %} to determine the actual hand position of the real robot (see the [observers' tutorial]({{site.baseurl}}/tutorials/recipes/observers.html)).

```cpp
void UpdateWall::start(mc_control::fsm::Controller & ctl)
{
  // [...] read YAML configuration parameters

  // Get the estimated pose of the fingers of the JVRC1 robot
  // As estimated by the state observation pipeline 
  const auto & bodyPose = ctl.realRobots().robot(rName).bodyPosW(bName);
  // Move the wall position along the x axis to match the fingertip's position
  auto posW = ctl.robot(moveRobotName).posW();
  posW.translation().x() = bodyPose.translation().x();
  ctl.robot(moveRobotName).posW(posW);
}
```

In the next state's, we will use this updated wall position to ensure collision avoidance with the hand while going back to the initial posture. 

### Going back to the initial posture

The remainder of the FSM is dedicated to getting the robot back to its initial configuration. First, to ensure a smooth transition, we release the pressure on the hand with the `RightHandReleaseAdmittance` until it reaches close to zero force, and then move the hand `10cm` backwards relative to its current position. Finally, we use the {% doxygen mc_control::fsm::HalfSittingState %} state to go back to the initial halfsitting posture. As we do not explicitly specify the hand motion, there is a risk that the hand could collide with the wall while going back to the half-sitting posture. To prevent this, we add a collision constraint between the hand and the wall:

```cpp
  RightHandMoveBack:
    base: StandingBase
    AddCollisionsAfter:
      - r1: jvrc1
        r2: wall
        collisions:
          - body1: R_WRIST_Y_S
            body2: ground
            iDist: 0.15
            sDist: 0.05
            damping: 0.0
    tasks:
    # [...]
```
