---
layout: tutorials
---

The framework provides by default a few pre-implemented states intended to make it easy to take full advantage of the `JSON/YAML` configuration capabilities of the FSM. Using these, writing FSMs for complex robot behaviours becomes easy, and usually involves little to no handwritten `C++/Python` states: almost everything can be directly described in the FSM configuration. In this tutorial, we'll explore use of the three most useful states:

- [MetaTasks]({{site.baseurl}}/json.html#State/MetaTasks): Loads a set of tasks from their `JSON/YAML` configuration
- [Parallel]({{site.baseurl}}/json.html#State/Parallel): Runs multiple states in "parallel"
- [Meta]({{site.baseurl}}/json.html#State/Meta): Nests an FSM within an FSM

In the remainder of this tutorial, we'll see how these may be used. Only `YAML` examples will be shown, `JSON` is of course also supported and similar examples may be found in the documentation. A full list of available states can be found in the ["State objects"]({{site.baseurl}}/json.html#State) section of the JSON/YAML documentation.

## MetaTasks state

The `MetaTasks` state ([documentation](json.html#State/MetaTasks)) allows to load a list of tasks from their `JSON/YAML` configuration. The full list of available tasks is available [JSON/YAML documentation]({{site.baseurl}}/json.html#MetaTask).

Let's start with an example, we will add a new `MetaTasks` state in the `states:` section of the `FSM` configuration.

```yaml
states:
  ExampleState:
    base: MetaTasks
    # optional: uses the output criteria of the tasks as the string output for the next transition
    outputs: [CoM]
    tasks:
      # create a CoM task
      CoM:
        type: com
        robotIndex: 0
        move_com: [0, 0, -0.05]
        completion:
          OR:
            - timeout: 3
            - AND:
              - eval: 0.01
              - speed: 0.005

      # create a bspline_trajectory task
      HandTrajectory:
        type: bspline_trajectory
        surface: LeftHand
        robotIndex: 0
        stiffness: 10000.0
        duration: 15.0
        weight: 100
        targetSurface:
          robotIndex: 1
          surface: Left
        completion:
          - timeElapsed: true
```

This will create a new state named `ExampleState` that will itself loads:
- A `CoM` task named `CoM` that moves the com `5cm` down.
- A `BSplineTrajectory` task named `LeftHandTrajectory` that moves the left gripper of the robot to a surface `Left` defined on another robot.

### Completion criterias

Notice the `completion` elements in the example above. These are what we refer to as "completion criteria". They are a way to generate conditional statements to determine when a task has been completed. See the API documentation for {% doxygen mc_control::CompletionCriteria %} for details.

By default, every task defines the following entries:

- `eval`: True when the norm of MetaTask::eval() is below the provided value
- `speed`: True when the norm of MetaTask::speed() is below the provided value
- `timeout`: True when the task has been added to the solver longer than the provided time (in seconds)

Some tasks define additional completion criterias, such as:
- `SplineTrajectoryTask` variants (`bspline_trajectory`, `exact_cubic_trajectory`) additionally define
  - `timeElapsed: true`: True when the task `duration` has elapsed
  - `wrench`: True when the force applied on the robot surface is higher than the provided threshold (6d vector, NaN value ignores the reading, negative values invert the condition). Ignored if the surface has no force-sensor attached.

You may add your own completion criterias by implementing `mc_task::MetaTask::buildCompletionCriteria` for your task. These completion criterias can be combined using the conditional constructs `AND` and `OR` as follows. The usual rules of lazy evaluation apply. For example, to check whether the hand trajectory tasks has completed if its total duration has been reached or more than 15N apply along the `z` direction of the hand's surface normal after 3 seconds of delay, you could write:

```yaml
# completion criteria that checks whether a trajectory task has been active for at least its specified duration
# or whether more than 15N apply along the `z` direction of the hand surface after 3 seconds of delay.
completion:
  OR:
    - timeElapsed: true
    - AND:
      - timeout: 3
      - wrench: [.NaN, .NaN, .NaN, .NaN, .NaN, 15]
```

You may also ommit the `completion` criteria for tasks that you want to execute but whose completion does not matter. In that case the FSM will consider the task as always completed.

### State output

As mentioned in the previous tutorials, each state returns an output string by calling `State::output("output value")`. This output value is used in the transition map to determine which state should be exectuted next. By default, the `MetaTasks` state returns `"OK"` once all of its tasks have completed. If you don't need to take branching decisions based on the tasks outputs, then your transition map would look like

```yaml
transtions:
...
# transition to the next state automatically when all tasks have completed
- [ExampleState, OK, NextState, Auto]
...
```

You may however want to make more complex branching decisions based on why the tasks have completed. For this, the `MetaTasks` state defines an `output` configuration which may contain a list of tasks names for which the completion criteria's string output will be used to generate the state's output. For example:

```yaml
ExampleState:
  base: MetaTasks
  # optional: uses the output criteria of the tasks as the string output for the next transition
  outputs: [CoM]
```

Your transition map can now branch conditionally based on the `CoM` completion criteria:

```yaml
# Reminder: the CoM completion criteria is defined as:
# CoM:
#   completion:
#     OR:
#       - timeout: 3
#       - AND:
#         - eval: 0.01
#         - speed: 0.005
- [ExampleState, "CoM=timeout", CoMHasNotConvergedState, Auto]
- [ExampleState, "CoM=eval AND speed", NextMotionState, Auto]
# state to execute by default if none of the completion patterns are matched in the transition map
# This allows to define non-exhaustive matching patterns in the transition map
- [ExampleState, "DEFAULT", DefaultState, Auto]
```

You may now define three new states:
- `CoMHasNotConvergedState` to be executed if the CoM task hasn't converged in the allowed time
- `NextMotionState` to continue moving since the CoM converged
- `DefaultState` to be executed in case no transition matches the generated output pattern. In this example the matching is exhaustive, and this state would never be executed.


## Parallel state

The parallel state ([documentation](json.html#State/Parallel)) allows to execute multiple states in parallel. Since the FSM execution is actually single-threaded, the states are actually executed sequentially in a single controller iteration.

```yaml
# Define an additional MetaTasks state that moves the right hand
RightHandState:
  base: MetaTasks
  HandTrajectory:
    type: bspline_trajectory
    surface: RightHand
    robotIndex: 0
    stiffness: 10000.0
    duration: 15.0
    weight: 100
    targetSurface:
      robotIndex: 1
      surface: Left
    completion:
      - timeElapsed: true

# Now say we want to move both the left and right hand and move the CoM down?
# Easy, simply put the two states in parallel:
ExampleParallelState:
  base: Parallel
  # At each iteration, the ExampleState will be executed, followed by the RightHandState
  states: [ExampleState, RightHandState]
  # optional defines which state to use as the output criteria
  # By default the last state's output in the states list above is used
  outputs: [ExampleState, RightHandState]
```

Parallel states are considered completed when all of its states have completed. As for the `MetaTasks` state, it is possible to specify how the state output is generated:
- By default, the output of the last state in `states` list will be used.
- When `outputs` is specified, these states' outputs will be used to generate the `parallel state`'s output.


For example:

```yaml
[ExampleParallelState, "ExampleState: (CoM=timeout), RightHandState: (timeElapsed)", "StateA"]
[ExampleParallelState, "ExampleState: (CoM=eval AND speed), RightHandState: (timeElapsed)", "StateB"]
# Here the interest of having a default state becomes apparent,
# as for complex condition criterias it might be cumbersome to create an exhaustive list of possible outputs.
[ExampleParallelState, "DEFAULT", "DefaultState"]
```

### Interacting between parallel states

Sometimes it is useful to be able to interact between states. For instance consider a `StabilizerStandingState` in parallel to a `Pickup` state that wants to pick-up an object on the floor. This state needs to move the `CoM` lower to enable the robot to reach. This kind of interaction between states can be achieved using the [DataStore]({{site.baseurl}}/tutorials/recipes/datastore.html).


## Meta: FSM within an FSM

In complex scenarios, it is often useful to be able to have multiple FSMs in the same controller. That's what the `Meta` state ([documentation](json.html#State/Meta)) is for, it embeds an `FSM` within an `FSM` state.

```yaml
ExampleMetaState:
  base: Meta
  transitions:
  - [StateA, OK, StateB]
  - [StateB, OK, StateC]
  - ...
```

This state can then be exectued as part of another FSM, or even as part of another `Meta` state.

```yaml
transtions:
- [ExampleParallelState, DEFAULT, ExampleMetaState]
- [ExampleMetaState, OK, LastStateOutput]
```

 Note that the ouput of a Meta FSM is the output of its last state without a transition within the Meta FSM (here `LastStateOutput`).
