In this tutorial,  we will re-implement the controller we implemented in the [multi-robot controller]({{site.baseurl}}/tutorials/introduction/multi-robot-controller.html) tutorial using only the FSM configuration. In this tutorial, we will walk you through every step needed to reproduce the `DoorSample` controller provided along with the framework from scratch.

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/GAtDC79G1zA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

Setting up the FSM
==

<div class="no_toc_section">

<ul class="nav nav-tabs" id="createTab" role="tablist">
  <li class="nav-item">
    <a class="nav-link active" id="cppCreateTab" data-toggle="tab" href="#cppCreateTabContent" role="tab" aria-controls="cppCreateTabContent" aria-selected="true">C++</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="pythonCreateTab" data-toggle="tab" href="#pythonCreateTabContent" role="tab" aria-controls="pythonCreateTabContent" aria-selected="false">Python</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="githubCreateTab" data-toggle="tab" href="#githubCreateTabContent" role="tab" aria-controls="githubCreateTabContent" aria-selected="false">GitHub</a>
  </li>
</ul>
<div class="tab-content" id="interfaceTabContent">
  <div class="tab-pane show active" id="cppCreateTabContent" role="tabpanel" arial-labelledby="cppCreateTab">
    <div class="card bg-light">
      <div class="card-body">
        {% translate_file tutorials/recipes/fsm-example/create-cpp.html %}
      </div>
    </div>
  </div>
  <div class="tab-pane" id="pythonCreateTabContent" role="tabpanel" arial-labelledby="pythonCreateTab">
    <div class="card bg-light">
      <div class="card-body">
        {% translate_file tutorials/recipes/fsm-example/create-python.html %}
      </div>
    </div>
  </div>
  <div class="tab-pane" id="githubCreateTabContent" role="tabpanel" arial-labelledby="githubCreateTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>Use the <a href="https://github.com/mc-rtc/new-fsm-controller">mc-rtc/new-fsm-controller</a> template project. This is equivalent to using the <code>mc_rtc_new_fsm_controller</code> tool with extra goodies.</p>
      </div>
    </div>
  </div>
</div>

</div>

You should now see the JVRC1 robot standing in RVIZ. Let's now see how to re-implement the [multi-robot controller]({{site.baseurl}}/tutorials/introduction/multi-robot-controller.html) tutorial using the FSM features. For this, you will need to edit your FSM configuration `etc/MyFirstFSMController.yaml`.

Loading the additional robots
==

First we need to declare which robots will be used in this controller in addition to the main robot. By default the framework provides many robots and environments, which can be conveniently loaded with robot aliases. These provide a short name for each robot, and provides mc_rtc with the necessary information for loading then (path to the robot description package, etc). Start the controller to see a full list of available robot aliases. Alternatively, you can look into <code>/usr/local/lib/mc_robots/aliases/</code>. For the purposes of this controller, we need a fixed planar surface to represent the ground provided as `env/ground`, and an articulated robot to represent the door with its handle provided as `env/door`. Refer to the [environment creation tutorial]({{site.baseurl}}/tutorials/advanced/new-environment.html) for details on how to create your own environment.

```yaml
robots:
  ground:
    module: env/ground
  door:
    module: env/door
    init_pos:
      translation: [0.70, 0.5, 0.0]
      rotation: [0.0, 0.0, 1.57]
```

Adding global contacts and constraints
==

Constraints
===

Constraints can be added globally to the FSM. Please refer to the [ConstraintSet JSON Schema]({{site.baseurl}}/json-full.html#ConstraintSet/ContactConstraint) documentation for a full list of available constraints provided with the framework. Note that you can also declare your own constraints and load them here.

```yaml
constraints:
- type: contact
- type: dynamics
  damper: [0.1, 0.01, 0.5]
- type: compoundJoint
```

Here we declare three constraints:
- `contact`: Adds an {% doxygen mc_solver::ContactConstraint %}, responsible for keeping the contact position fixed, and ensures that the generated forces are within friction cone constraints (in dynamics only).
- `dynamics`: Adds a {% doxygen mc_solver::DynamicsConstraint %}: ensures kinematic constraints, joint limit constraints, and computes the joint torques.
- `compoundJoint`: Adds a {% doxygen mc_solver::CompoundJointConstraint %}: handle joint limits for joint that depend on each other (e.g the ankle joint limits in the roll direction depend on the current yaw angle)


Contacts
===

Now let's see how to add an initial set of contacts to the controller. Note that contacts can later be added/removed by the states. The `contacts` element contains an array of [Contact]({{site.baseurl}}/json-full.html#mc_rbdyn/Contact) which describe the contact properties to add to the `contact` constraint defined above.

```yaml
# Initial set of contacts
contacts:
- r1: jvrc1
  r1Surface: LeftFoot
  r2: ground
  r2Surface: AllGround
- r1: jvrc1
  r1Surface: RightFoot
  r2: ground
  r2Surface: AllGround
```

With this, the left and right foot surfaces are now considered in contact with the ground plane. The feet surfaces are constrained not to move, and the forces generated by the QP must remain within the linearized friction cone constraints.

Collisions
===

Let's now see how to add an initial set of collisions. As is the case for the contacts, those may later be added/removed by the states. The `collisions` element contains an array of [CollisionConstraint]({{site.baseurl}}/json-full.html#ConstraintSet/CollisionsConstraint) objects. For the `MainRobot`, a default list of self-collisions is defined in the {% doxygen mc_rbdyn::RobotModule %} and may be used here.

```yaml
# Collision constraint
collisions:
- type: collision
  useMinimal: true  # The set of minimal self-collisions is defined in the robot module
- type: collision
  r1: jvrc1
  r2: door
  collisions: # array of collisions to add for this pair of robots
    - body1: L_WRIST_Y_S
      body2: door
      iDist: 0.5  # interaction distance: minimal distance below which the constraint becomes active
      sDist: 0.02 # safety distance: minimal allowed distance
      damping: 0.0
```

Creating the FSM states
===

In this section, we will see how to define the states required to open the door. Here we will take full advantage of the default `C++` states provided with the framework, and show how using those, one may achieve rather complex behaviours without ever writing a single line of code! This controller uses the following (very common) states:

- `MetaTasks`: Loads tasks from configuration, add them to the solver, and check for tasks completion
  - API: {% doxygen mc_control::fsm::MetaTasksState %}
  - YAML: [JSON Schema Documentation]({{site.baseurl}}/json.html#State/MetaTasks)
- `Parallel`: Run multiple states
  - API: {% doxygen mc_control::fsm::ParallelState %}
  - YAML: [JSON Schema Documentation]({{site.baseurl}}/json.html#State/Parallel)
- `Meta`: Creates an FSM within a state
  - API: {% doxygen mc_control::fsm::MetaState %}
  - YAML: [JSON Schema Documentation]({{site.baseurl}}/json.html#State/Meta)
- `Posture`: Handles the global posture tasks' gains and targets
  - API: {% doxygen mc_control::fsm::PostureState %}
  - YAML: [JSON Schema Documentation]({{site.baseurl}}/json.html#State/Posture)

The states are declared within the `states` section of the configuration. Alternatively, one may also create a `.yaml` file in `src/states/data` containing the states declaration, which is particularly useful for larger FSMs.

We will create an FSM organized as follow:
1. `Door_Initial`: A simple C++ state that adds a button to trigger the "Open Door" motion
2. `Door::OpenDoorFSM`: A sub-fsm containing only the logic for moving the hand, opening the handle, and moving the door
3. `Door::Standing`: A state responsible for adding a `CoM` task, and a regularisation task on the chest to keep it upright
4. `Door::OpenDoorDemo`: States `2` and `3` are put in parallel such that the CoM/Chest target is handled simultaneously to the door opening motion.

We fully exploit the multi-robot aspect, and the door will be moved by establishing a contact between the hand and the handle, and controlling the joint-angle of the handle and door hinge. Due to the contact constraint, the QP will automatically generate the necessary motion required to achieve this motion.

Initial State: Adding a button to trigger a transition
====

This state aims at demonstrating a simple example of:

1. How to create a C++ state
2. How to add a GUI element within a state
3. How to control the flow of transitions from within a state

This state is defined in `src/states/Door_Initial.cpp` and overrides the required virtual functions `configure`, `start` and `run` and `teardown` function (see the API documentation of {% doxygen mc_control::fsm::State %}). Here, we add a button to the GUI when the state starts. When clicked, this button is used to change the state of a boolean `openDoor_`, which is later used to trigger an "OpenDoor" transition. This is achieved by calling `output("OpenDoor")` and returning `true` in the `bool Door_Initial::run` function, which signifies that we consider the state to be completed and that the next transition can occur. Note that the the configuration of the transition map determines how this transition occurs (see {% doxygen mc_control::fsm::TransitionMap %}).

```cpp
#include <mc_control/fsm/Controller.h>

#include "Door_Initial.h"

void Door_Initial::configure(const mc_rtc::Configuration &) {}

void Door_Initial::start(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->addElement({}, mc_rtc::gui::Button("Open door", [this]() { openDoor_ = true; }));
}

bool Door_Initial::run(mc_control::fsm::Controller &)
{
  if(openDoor_)
  {
    output("OpenDoor");
    return true;
  }
  return false;
}

void Door_Initial::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("Door_Initial", Door_Initial)
```

The `EXPORT_SINGLE_STATE` macro is used to declare the loading symbols used by mc_rtc to load the state from library. The name `"Door_Initial"` provided here is used to identify this state within the FSM.

Note that here this state is very simplistic on purpose. In practice, states typically perform more complex actions, such as adding tasks and handling their targets, reading data from planners, monitoring the state of the robot and triggering transitions, etc. Examples of such states are the `MetaTasks`, `Parallel`, `Meta` and `Posture` tasks used within this example.

FSM for opening the door
====

Opening the door is achieved by a state executing its own sub-fsm, responsible for handling the hand's motion towards the door handle, establishing contact with it, and then turning the handle and door hinge to open it.

First, let's see what the transition map for this state looks like:

```yaml
Door::OpenDoorFSM:
  base: Meta
  transitions:
    - [Door_Initial, OpenDoor, Door::ReachHandle, Auto]
    - [Door::ReachHandle, OK, Door::MoveHandle, Auto]
    - [Door::MoveHandle, OK, Door::OpenDoor, Auto]
```

The `Door_Initial` C++ state described above will trigger the `OpenDoor` transition when the user clicks on the GUI button. The FSM will then move to the next state `Door::ReachHandle`:

```yaml
Door::ReachHandle:
  base: MetaTasks
  tasks:
    RightHandTrajectory:
      type: surfaceTransform
      surface: RightGripper
      weight: 1000
      stiffness: 5
      # Target relative to the door's handle surface
      targetSurface:
        robot: door
        surface: Handle
        offset_translation: [0, 0, -0.025]
        offset_rotation: [0, 0, 0]
      completion:
        AND:
          - eval: 0.05
          - speed: 1e-4
```

This state uses the {% doxygen mc_control::fsm::MetaTasksState %} C++ state provided with the framework to load a set of tasks from their YAML description. Here, we load a task of type `surfaceTransform` ([YAML Documentation]({{site.baseurl}}/json.html#MetaTask/SurfaceTransformTask)) that we name `RightHandTrajectory`. This creates and adds an {% doxygen mc_tasks::SurfaceTransformTask %} to the solver, and configures it with a target defined w.r.t the door's handle surface. The `completion` element creates a {% doxygen mc_control::CompletionCriteria %}, which builds a logic function that checks whether the task's execution is considered completed. The `MetaTasks` state will output `"OK"` (by default) when the task's completion criteria is fulfilled.

The `OpenDoorFSM` can thus move to its next transition: `[Door::ReachHandle, OK, Door::MoveHandle, Auto]`

```yaml
Door::MoveHandle:
  base: Posture
  robot: door
  completion:
    eval: 0.01
  postureTask:
    weight: 100
    jointGains:
      - jointName: handle
        stiffness: 50
    target:
      handle: [-1.0]
  AddContacts:
  - r1: jvrc1
    r1Surface: RightGripper
    r2: door
    r2Surface: Handle
```

This state is based on the {% doxygen mc_control::fsm::PostureState %} ([JSON documentation]({{site.baseurl}}/json.html#States/Posture)) whose role is to change gains and targets of the global posture task, automatically added to each robot by the framework. First, a contact between the robot's `RightGripper` and the door's `Handle` surfaces is established which prevents the QP from moving these surfaces relative to each other, and adds friction cone constraints to compute the dynamical forces involved in the robot-door interaction. Thus, when the handle joint rotates so does the JVRC1 robot.

Once this state completes, we can move to the next transition: `[Door::MoveHandle, OK, Door::OpenDoor, Auto]`. This next state is very similar to the previous one: it changes the target joint angle of the door's hinges to make the robot open it. Note how this state inherits from the previous one (`base: Door::MoveHandle`), and only redefines the new targets.

```yaml
Door::OpenDoor:
  base: Door::MoveHandle
  postureTask:
    jointGains:
      - jointName: handle
        stiffness: 50
      - jointName: door
        stiffness: 50
    target:
      handle: [-1.0]
      door: [-0.3]
```


As-is, this FSM only concerns itself with the door opening motion. However, it does not handle the robot's balance. Here, we will simply center the CoM above the left and right foot. This is achieved by putting the `Door::Standing` state in parallel with the `Door::OpenDoorFSM` described above. Strictly speaking these states are executed one after the other at every timestep.

```yaml
Door::OpenDoorDemo:
  base: Parallel
  states: [Door::Standing, Door::OpenDoorFSM]
```

with the standing state defined as:

```yaml
Door::Standing:
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

The full sources for this tutorial are available [here](https://github.com/jrl-umi3218/mc_rtc/tree/master/src/mc_control/samples/Door).

Conclusion
===

In this tutorial, we have seen how to create an FSM from scratch, and achieve a rather complex multi-robot motion by relying on the main FSM states provided by the framework, along with the multi-robot aspect of task-space control. It is important to note that one is not restricted to the YAML features used here, and that you can easily write your own states to define and abstract more complex behaviours.

See also:
- The [Admittance sample tutorial]({{site.baseurl}}/tutorials/samples/sample-admittance.html) for a similar FSM with the addition of force control.
