A finite-state machine (FSM) is an abstract machine that can be in one of a finite set of states. Based on conditions internal or external to the current state, the machine's state can change. Such a state is known as a transition. Such machines are ubiquitous in programming and are particularly well-suited to implement robotic scenarios due to the simplicity and the composability it offers.

The state chart formalism extends the finite state machine formalism and introduce two key principles:
- **Hierarchy** allows to nest a state chart within a state chart;
- **Concurrency** (also known as orthogonality) allows the parallel execution of two or more states.

mc\_rtc provides a controller implementation to implement the state chart formalism. This page covers the ins and outs of this implementation to help you work with it.

## State, transitions and FSM run

### State

We will discuss the state implementation later. For now we will simply describe the four main methods of a state:

- `configure` is used to configure the state; it is called multiple times
  - once for every level of inheritance in the state hierarchy;
  - once with the executor configuration, i.e. from the `configs` entry in the global FSM or a state that manages other state (e.g. `Meta` or `Parallel`);
  - the default implementation simply loads the successive configuration passed to the function into a `config_` object that can be accessed in your state implementation; you can override this function if you want to implement a more complex loading scheme;
- `start` is used to perform initialization; it is called only once;
- `run` is the main function implemented by a state; it is called once per iteration loop until the state is over or until the state changes. When the run is completed, the state will set an output to an arbitrary value that must be documented by the state;
- `teardown` is a cleanup function that is called when the state changes.

### Transition

A transition is formed by a 4-uplet representing:

1. The transit-from state;
2. The output of the state;
3. The transit-to state;
4. An optional parameter that influences how the FSM handles the transition. The possible values are: `StepByStep` (default), `Auto` and `Strict`, we will see their respective meaning shortly.

### FSM run

The FSM controller can run in two distinct modes: managed and non-managed. We will focus first on the non-managed mode.

#### Non-managed mode

In this mode, the FSM takes care of the state creation and execution as well as transitions between the states.

One iteration of the FSM uses the following logic:

* A state is already running
  * Run the state run method
  * If its returns true, check the state output and find the associated transition
    * Teardown the state
    * If the transition type is `Auto` or if it is `StepByStep` and the `StepByStep` setting of the FSM is `false`
      * Configure and initialize the next state
    * Else
      * Setup idle state
* A state is not running
  * A transition has been triggered
    * Undo idle state, configure and initialize the next state

A few details were left out of this overview to simplify the logic flow:
1. State execution can be interrupted by an external trigger. In this case, before anything else, the state is teardown and the FSM waits for a transition command;
2. If no transition is available for a given pair of state/output, the FSM considers it completed its run. It can be restarted by sending a transition command;
3. Idle state is optional, if it is disabled in the case when the idle state would be started, the previous state run method continues being called until the transition is triggered.

##### About idle state

Idle state (when enabled) attempts to leave the robot in the state that was last achieved by the previously running state. To do so it uses two tasks:
- A posture task where the objective is set to the current posture;
- An end-effector task for the free-flyer where the object is set to the current attitude. This task is not active for fixed-base robots.

This "state" will always be used when an interruption has been triggered.

#### Managed mode

There is not much to say about this mode. In managed mode, the FSM does not take care of transitions, this is strictly handled by an external tool. A state lifetime is similar to that described above but **all** transitions are triggered by an external tool.

## Text-based inheritance

The states we describe here are C++ objects and they provide many configuration options. However, it would be tedious to repeat the complete set of options when one wants to define two (or more) subtly different states. Therefore, the interface provides a way to inherit configuration set.

Assuming we have a C++ state named `StateBase`, we can do the following:

```yaml
MyFirstState: # <-- Name of the new state
  base: StateBase # <-- Name of the C++ state we are based on
  # other options
MySecondState: # <-- Name of the new state
  base: MyFirstState # <-- This time we use a state we defined before only in text form
  # other options
```

The way options are combined depends on the C++ state implementation and should be documented by it. However, the general rule applies for mc_rtc provided states and they are based on the defalt loading rule for `mc_rtc::Configuration` objects:
- "raw" values (boolean, numbers and strings) and vectors are overwritten
- objects are merged following the rule:
  - if the key does not exist in the destination object, the source object value is used
  - if the key exists in both and their types match, the rules are applied (recursively in case of an object)
  - otherwise, the source object value overwrite the destination object value

## States already implemented in mc_rtc


This section covers the main states that are provided as part of mc_rtc. A complete description of all available states and their configuration can be found in the [States JSON Schema]({{site.baseurl}}/json.html#State-objects).

<div class="no_toc_section">


<ul class="nav nav-tabs" id="statesTab" role="tablist">
  <li class="nav-item">
    <a class="nav-link active" id="PauseTab" data-toggle="tab" href="#PauseTabContent" role="tab" aria-controls="PauseTabContent" aria-selected="true">Pause</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="MetaTasksTab" data-toggle="tab" href="#MetaTasksTabContent" role="tab" aria-controls="MetaTasksTabContent" aria-selected="false">MetaTasks</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="AddRemoveContactTab" data-toggle="tab" href="#AddRemoveContactTabContent" role="tab" aria-controls="AddRemoveContactTabContent" aria-selected="false">AddRemoveContact</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="ParallelTab" data-toggle="tab" href="#ParallelTabContent" role="tab" aria-controls="ParallelTabContent" aria-selected="false">Parallel</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="MetaTab" data-toggle="tab" href="#MetaTabContent" role="tab" aria-controls="MetaTabContent" aria-selected="false">Meta</a>
  </li>
</ul>
<div class="tab-content" id="statesTabContent">
  <div class="tab-pane show active" id="PauseTabContent" role="tabpanel" arial-labelledby="PauseTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>This states simply waits for a while and outputs <code>OK</code>.</p>

        <h5>Options</h5>

        <ul>
          <li><code>duration</code>: duration of the pause in seconds as a floating point value, defaults to 0.</li>
        </ul>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="MetaTasksTabContent" role="tabpanel" arial-labelledby="MetaTasksTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>This state creates an arbitrary number of MetaTasks and executes them until completion criteria are fullfiled for each tasks.</p>

        <p>The tasks in the state are configured through the <code>tasks</code> entry which is a JSON object.</p>

        <p>The keys are the name of the tasks and values are MetaTask objects as expected by MetaTaskLoader plus an optional "completion" criteria that represents one or more completion criteria.</p>

        <p>The names of the tasks are only relevant for the state. You can override the actual task's name using the name entry in the Task's configuration.</p>

        <p>If the <code>completion</code> entry is absent and if there is no existing completion entry for the related task then this task is added but not considered as part of the completion criteria (e.g. you can add a CoMTask and an EndEffectorTask but only care for the completion of the later).</p>

        <p>When the <code>tasks</code> entry is read multiple times, the following ensues:</p>
        <ul>
        <li>if a new tasks appears then it is added</li>
        <li>if an existing task is not repeated, nothing happens for this task</li>
        <li>if a task already exists, existing configuration entries are overwriten by the new entry, non-existing configuration entries are simply added to the existing configuration</li>
        </ul>

        <h5>Example</h5>

        {% highlight yaml %}
        # We simplify task entries for the sake of the example

        # First pass
        tasks:
          t1:
            objectiveA: 0.5,
            objectiveB: 1.0,
            completion: { timeout: 5.0 }

        # After this pass, one task is considered

        # Second pass
        tasks:
          t1:
            objectiveA: 1.0,
            completion: { eval: 1e-6 }
          t2:
            objective: 0.5

        # We now have two tasks, and:
        # - t1's objectiveA is changed to 1.0, objectiveB is the same
        # - t1 completion criteria is replaced

        # Third pass
        tasks:
          t1:
            completion: {}
          t2:
            completion: { eval: 1e-6 }

        # We still have two tasks, objectives are unchanged but:
        # - t1 has no more completion criteria
        # - t2 has a completion criteria
        {% endhighlight %}

        <h5>Options</h5>

        <ul>
          <li><code>tasks</code>: object describing the tasks to add to the controller for this state;</li>
        </ul>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="AddRemoveContactTabContent" role="tabpanel" arial-labelledby="AddRemoveContactTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>Implements a state that is able to remove or add a contact.</p>

        <h5>Options</h5>

        <p>The folowing two entries are required to configure the state:</p>

        <ul>
          <li><code>type</code>: one of [<code>addContact</code>, <code>removeContact</code>, <code>compliance</code>]</li>
          <li><code>contact</code>: contact to be removed or added</li>
        </ul>

        <h5>Remove contact options</h5>

        <ul>
          <li><code>distance</code>: when the contact body has moved this distance away from the contact, the state is finished. Default is 0.1 (10 cm).</li>
        </ul>

        <h5>Compliant add contact options</h5>

        <ul>
          <li><code>velocity</code>: velocity threshold for the ComplianceTask. Default is 1e-4.</li>
        </ul>

        <p>Other options depend on the type of task used to add/remove the contact. For the compliance task, the <code>body</code> entry is overwritten based on the contact value.</p>

        <p>If you use a compliance task to remove a contact that has no force sensor attached the state will automatically fallback to <code>addContact</code>.</p>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="ParallelTabContent" role="tabpanel" arial-labelledby="ParallelTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>Implements parallel states.</p>

        <p>This states plays multiple states at once. Strictly speaking those states are not played in parallel but rather sequentially.</p>

        <p>If this state plays <code>{state_1, ..., state_N}</code>. The state is completed when all <code>state_i::run()</code> function returns true and its output is <code>state_N</code> output.</p>

        <h5>Options</h5>

        <ul>
          <li><code>states</code>: list of states run by this state</li>
          <li><code>configs</code>: for each state in states, configs(state) is used to further configure the states</li>
        </ul>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="MetaTabContent" role="tabpanel" arial-labelledby="MetaTab">
    <div class="card bg-light">
      <div class="card-body">
        <p>Implements a "meta" state.</p>

        <p>This states plays its own FSM.</p>

        <h5>Options</h5>

        <ul>
          <li><code>Managed</code>: if true, does not handle transitions</li>
          <li><code>transitions</code>: a transition map, similiar to the FSM controller (required if Managed is false)</li>
          <li><code>StepByStep</code>: same as FSM for the internal FSM (default to the parent FSM <code>StepByStep</code> setting)</li>

          <li><code>configs</code>: can contain additional configuration for the states in the FSM</li>
        </ul>
      </div>
    </div>
  </div>
</div>

</div>

### Common options

Some options are common to all states:

- `AddContacts`/`RemoveContacts`: allows to add/remove contacts **before** the state is executed, this should be a vector of `fsm::Contact` objects;
- `AddContactsAfter`/`RemoveContactsAfter`: allows to add/remove contacts **after** the state is executed, this should be a vector of `fsm::Contact` objects;
- `RemovePostureTask`: if set to true, the default posture tasks will be removed.


## New state creation

A state is created by inheriting from {% doxygen mc_control::fsm::State %}. Its minimal interface is the following:

```cpp
namespace mc_control::fsm
{
  struct MyState : public State
  {
    void configure(mc_rtc::Configuration &) override;

    void start(Controller &) override;

    bool run(Controller &) override;

    void teardown(Controller &) override;
  };
}

EXPORT_SINGLE_STATE("MyState", mc_control::fsm::MyState);
```

In all functions where a `Controller` instance is passed it is an {% doxygen mc_control::fsm::Controller %} instance.

### `void configure(mc_rtc::Configuration &)`

This function is called **multiple** times. It is very important to remember that as you should only "accumulate" configuration entries in this function and not create any tasks.

### `void start(Controller &)`

This function is called once to initialize the state. This is where you'll want to transform your accumulated configuration into a runnable state.

### `bool run(Controller &)`

This function will be called on every iteration following the `start` iteration, i.e. there will be no call to `run` in the same iteration as `start` unless you manually call `run` from your `start` implementation.

This function should return `true` when the state is completed. At this point you should also set the state's output by calling `output(const std::string &)`.

### `void teardown(Controller &)`

This function will be called once before the state is destroyed. This should be used to cleanup the effect of the state on the controller.

### FSM controller specificities

The FSM has some differences with regular mc_rtc controllers.

#### Contacts

The FSM uses a lighter form of the contact structure which has the following definition:

```cpp
struct Contact
{
  std::string r1;
  std::string r2;
  std::string r1Surface;
  std::string r2Surface;
  Eigen::Vector6d dof; // defaults to Eigen::Vector6d::Ones()
};
```

The `dof` vector will be transformed to a diagonal matrix and added as a [dof constraint](contact-dof.html).

To add/remove contacts in the FSM controller, simply call:

```cpp
void addContact(const Contact &);

void removeContact(const Contact &);
```

#### Collisions

To add/remove collisions, simply call:

```cpp
void addCollisions(const std::string & r1, const std::string & r2,
                   const std::vector<mc_rbdyn::Collision> & collisions);

void removeCollisions(const std::string & r1, const std::string & r2,
                   const std::vector<mc_rbdyn::Collision> & collisions);

// Remove all collisions between r1 and r2
void removeCollisions(const std::string & r1, const std::string & r2);
```

The FSM will create and add the necessary collision constraints if necessary.

#### Posture tasks

The FSM controller creates a posture task for every actuated robot. You can access this task by calling:

```cpp
std::shared_ptr<mc_tasks::PostureTask> getPostureTask(const std::string & robot);
```

This task is normally kept in the solver during the state's lifetime. If this is not desired, you can remove it and put it back in the `teardown` call.

### Other methods

The following methods are virtual in the `State` interface and can be optionally overriden.

#### `void stop(Controller &)`

This is called if the state is interrupted.

## FSM configuration

The following options can be used to configure the FSM:

- `Managed`: if true, the FSM is managed, otherwise it is not;
- `StepByStep`: if true, transitions that are tagged as `StepByStep` will behave as `Strict` transitions, otherwise they behave as `Auto` transitions;
- `IdleKeepState`: if true, the state is kept alive until the transition is triggered by the user;
- `StatesLibraries`: where to look for states libraries;
- `StatesFiles`: where to look for states configuration files;
- `VerboseStateFactory`: if true, the state factory will provide more information while loading libraries, this is useful for debugging;
- `robots`: JSON object, each key is the name of a robot and the value is an object representing a robot module to load in addition to the main robot module;

```json
// Example robots entry
"robots":
{
  "ground":
  {
    "module": "env",
    "params": ["@MC_ENV_DESCRIPTION@", "ground"]
  }
}
```

- `constraints`: array of constraints, each object is a JSON representation of a `mc_solver::ConstraintSet` object as specified by the JSON schemas;
- `collisions`: array of collision constraints following the `mc_solver::CollisionConstraint` sJSON schema;
- `contacts`: array of initial contacts;

```json
// Example contacts entry
"contacts":
[
  {
    "r1": "jvrc1",
    "r2": "ground",
    "r1Surface": "LeftFoot",
    "r2Surface": "AllGround"
  },
  {
    "r1": "jvrc1",
    "r2": "ground",
    "r1Surface": "RightFoot",
    "r2Surface": "AllGround"
  }
]
```

- for each robot, named <code>r</code>, an entry <code>r</code> can be used to configure the posture and free-flyer task usd in the idle state;
- `states`: object, each key is the name of a state, the value is this state configuration;
- `configs`: object, each key is the name of a state, the value is additional configuration to pass to this state when it is played in the main FSM;
- `transitions`: an array, each elements of the array is a transition
- `init`: the initial state to start the FSM;
- `init_pos`: initial position of the main robot (7d array);

In the next tutorial we will implement a practical example of the FSM controller.
