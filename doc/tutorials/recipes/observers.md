---
layout: tutorials
---

When dealing with robotic systems, it is common to need an estimation of the state of the system being controlled. Doing so requires writing some form of state observer, that takes as input sensor measurements (joint encoders, force-torque sensors, IMU, cameras, etc) and additional knowledge about the system (contacts, etc) and infers all or part of the system's state. To simplify this estimation process, the framework provide a facility refered to as the `Observers pipeline`. The main concept is to consider the observation of the full robot state as a sequence of simple estimators that each observe a specific part of the state (floating base position, encoder kinematics, etc) and update the robot state accordingly. The framework makes the distinction between two set of robot instances:

- `robots()` represents the control state (desired) of the system.
- `realRobots()` represents the state of the real system. The point of the `Observers pipeline` is to simplify the observation of its state.

For example, in case of a humanoid robot, one needs the full dynamic properties of the robot, that is the position, orientation and velocity of each body, including the floating base. Considering common sensors available on a humanoid robot, these can be estimated from encoder measurements and an IMU using a simple kinematic-inertial estimator. Using the facilities described in this tutorial, such as estimator can be achieved through the following pipeline:

```
Observers: Encoder (position=estimator,velocity=estimator) -> [BodySensor (sensor=FloatingBase,update=estimator)] -> KinematicInertial (cutoff=0.01)
```

Note that the above short description of the pipeline will be displayed when starting your controller. It provides a short description of the pipeline and its options, and reads as follows:

- Observers are executed in sequential order (separated by `->`)
- Relevant details of each observer configuration are shown between parenthesis `(...)`
- Observers shown between brackets (`[...]`) are run, but do not affect the state of the `realRobots()` instance. This allows to have several observers estimating the same state simultaneously, for comparison purposes. Here both `BodySensor` (groundtruth) and `KinematicInertial` estimate the state of the floating base, but only `KinematicInertial` observer result is used to update the `realRobots()` instances.

Each observer is allowed to modify properties of the `realRobots()` instance. In the above example:
- The `Encoder` observer will compute the encoder velocity through finite differences of the encoder position sensor, and use this estimate for the kinematic properties of the real robot instance (e.g set `realRobot().mbc().q, realRobot().mbc().alpha` and compute forward kinematics and forward velocity to update each body pose and velocity).
- Then the `BodySensor` observer will be run, and only compute the pose of the floating base from `BodySensor` information (typically groundtruth from a simulator). It will however not update any properties of the `realRobots()` instance.
- Finally, the `KinematicInertial` observer is run, and estimates the floating base pose and velocity (through filtered finite differences) based on IMU measurement. This estimate is in turn used to update the floating base (i.e `realRobot().posW(...)` and `realRobot().velW(...)`).

Thus in the end the `realRobot()` instance will contain a full estimate of the afformentioned state, that is floating base position and velocity, encoder position and velocity. This estimated robot is widely available throughout your controller, and allows you to access any property of the real system in the same way you would access the control instance. For example, you can access:

- Body pose: `realRobot().bodyPosW("bodyName");`
- Body velocity: `realRobot().bodyVelW("bodyName");`
- Compute the CoM position and velocity `realRobot().com() / realRobot().comVelocity()`
- ...

The end-result for this example pipeline looks like this *(left: choreonoid simulation, right: control state [transparent], observed state [solid])*

<div class="embed-responsive embed-responsive-16by9">
  <video src="https://gite.lirmm.fr/multi-contact/mc_rtc/uploads/bd05951bc324f2e367e9626f458371cb/screencast_observers.mp4" controls />
</div>

# Default observers

Several common observers are provided with the framework. Their role and requirements will be described in this section, along with a sample JSON/YAML configuration.

## Encoder

- **Prerequisites**: encoder position sensor
- **Run**: keeps the encoder position from sensor unchanged, computes encoder velocity by finite differences of the position sensor
- **Update**:
   - `UpdatePositionFrom`: updates `realRobot().mbc().q`
      - `"estimator"` :  from encoder position.
      - `"control"` : from `robot().mbc().q` values (eg copies the control state instead of using sensors).
      - `"none"`: does not update the encoder position
   - `"UpdateVelocityFrom":` : updates `realRobot().mbc().alpha`
      - `"estimator"`: from estimated encoder velocity.
      - `"control"`: from `robot().mbc().alpha` values (eg copies the control state instead of using sensors).
      - `"none"`: does not update the encoder velocity
- **Computes**: forward kinematics, forward velocity for the `estimator|control` updates.


<ul class="nav nav-tabs" id="createTab" role="tablist">
  <li class="nav-item">
    <a class="nav-link active" id="yamlCreateTab" data-toggle="tab" href="#yamlCreateTabContent" role="tab" aria-controls="yamlCreateTabContent" aria-selected="true">YAML</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="jsonCreateTab" data-toggle="tab" href="#jsonCreateTabContent" role="tab" aria-controls="jsonCreateTabContent" aria-selected="false">JSON</a>
  </li>
</ul>
<div class="tab-content" id="interfaceTabContent">
  <div class="tab-pane show active" id="yamlCreateTabContent" role="tabpanel" arial-labelledby="yamlCreateTab">
    <div class="card bg-light">
      <div class="card-body">
<p>
{% highlight yaml %}
Encoder:
  # Valid values are [estimator, control, none]
  UpdatePositionFrom: estimator
  UpdateVelocityFrom: estimator
  Log : true
{% endhighlight %}
</p>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="jsonCreateTabContent" role="tabpanel" arial-labelledby="jsonCreateTab">
    <div class="card bg-light">
      <div class="card-body">
<p>
{% highlight json %}
"Encoder":
{
  "UpdatePositionFrom": "estimator",
  "UpdateVelocityFrom": "estimator",
  "Log" : true
}
{% endhighlight %}
</p>
      </div>
    </div>
  </div>
</div>


## BodySensor

- **Prerequisites**:
  - Up to date kinematic information (typically obtained by the `Encoder` observer)
  - The `BodySensor` must contain position, orientation, linear and angular velocity
- **Run**:
  - If `"UpdateFrom": "estimator"`: Computes the floating base pose and velocity from a `BodySensor`. If the body sensor is not directly attached to the floating base link, the transformation between the sensor frame and the floating base frame is taken into account using their relative pose obtained by kinematics.
  - If `"UpdateFrom": "control"`: Use the control robot floating base (`robot().posW(), robot().velW()) instead
- **Update**:
  - sets the real robot floating base state
  - computes forward kinematics/velocity when necessary

<ul class="nav nav-tabs" id="createTab" role="tablist">
  <li class="nav-item">
    <a class="nav-link active" id="bodySensorYAMLCreateTab" data-toggle="tab" href="#bodySensorYAMLCreateTabContent" role="tab" aria-controls="bodySensorYAMLCreateTabContent" aria-selected="true">YAML</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="bodySensorJSONCreateTab" data-toggle="tab" href="#bodySensorJSONCreateTabContent" role="tab" aria-controls="bodySensorJSONCreateTabContent" aria-selected="false">JSON</a>
  </li>
</ul>
<div class="tab-content" id="interfaceTabContent">
  <div class="tab-pane show active" id="bodySensorYAMLCreateTabContent" role="tabpanel" arial-labelledby="bodySensorYAMLCreateTab">
    <div class="card bg-light">
      <div class="card-body">
<p>
{% highlight yaml %}
BodySensor:
  # Valid entries are [control, estimator]
  UpdateFrom: estimator
  FloatingBaseSensor": FloatingBase
{% endhighlight %}
</p>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="bodySensorJSONCreateTabContent" role="tabpanel" arial-labelledby="bodySensorJSONCreateTab">
    <div class="card bg-light">
      <div class="card-body">
<p>
{% highlight json %}
"BodySensor":
{
  "UpdateFrom": "estimator",
  "FloatingBaseSensor": "FloatingBase"
}
{% endhighlight %}
</p>
      </div>
    </div>
  </div>
</div>

## KinematicInertial

Observes the state of the robot's floating base (pose and velocity) based on IMU measurements, kinematics information, and a reference anchor point. A full explanation of the method implemented here can be found on [Stéphane Caron's website](https://scaron.info/teaching/floating-base-estimation.html). Note that this observer has been extensively used in practice in the [LIPM Walking Controller](https://github.com/jrl-umi3218/lipm_walking_controller).

- **Prerequisites**:
  - IMU sensor
  - Accurate kinematics (body position)
  - A known anchor frame, to be provided by calling

  ```cpp
  void mc_control::anchorFrame(const sva::PTransformd & );
  void mc_control::anchorFrameReal(const sva::PTransformd & );
  ```
 The anchor frame is a point in-between the robot feet used to estimate the position of the floating base from its orientation and the robot kinematics between the anchor sole and the floating base frame.

- **Run**
  - Computes the floating base pose from IMU, kinematics and the anchor frame
  - Computes the floating base velocity by finite differences of the estimated pose, filtered with a low-pass filter.

- **Update**
  - The floating base pose and velocity
  - Computes forward kinematics/veloctiy

**Default configuration**: no configuration options implemented


## Setting-up the observers pipeline

The observer pipeline can be configured directly from your controller's configuration file as follows.

<ul class="nav nav-tabs" id="createTab" role="tablist">
  <li class="nav-item">
    <a class="nav-link active" id="confYAMLCreateTab" data-toggle="tab" href="#confYAMLCreateTabContent" role="tab" aria-controls="confYAMLCreateTabContent" aria-selected="true">YAML</a>
  </li>
  <li class="nav-item">
    <a class="nav-link" id="confJSONCreateTab" data-toggle="tab" href="#confJSONCreateTabContent" role="tab" aria-controls="confJSONCreateTabContent" aria-selected="false">JSON</a>
  </li>
</ul>
<div class="tab-content" id="interfaceTabContent">
  <div class="tab-pane show active" id="confYAMLCreateTabContent" role="tabpanel" arial-labelledby="confYAMLCreateTab">
    <div class="card bg-light">
      <div class="card-body">
<p>
{% highlight yaml %}
# Defines which observer libraries will be loaded by the framework
EnabledObservers: [Encoder, BodySensor, KinematicInertial]
# List of observers to run
RunObservers: [Encoder, BodySensor, KinematicInertial]
# List of observers used to update realRobots() from
UpdateObservers: [Encoder, KinematicInertial]

# Specific configuration for each observer (optional)
Observers:
  Encoder:
    UpdatePositionFrom: estimator
    UpdateVelocityFrom: estimator
    Log : true

  BodySensor:
    UpdateFrom: estimator
    FloatingBaseSensor: FloatingBase

# These observers are loaded from libraries (similarely to the controllers and metatasks).
# The default path is `<INSTALL_PREFIX>/lib/mc_observers/ObserverName.so`.
# Optionally, you can specify additional paths for your observer libraries as
# ObserverModulePaths: ["/one/path/to/observer/", "/another/path/"]
{% endhighlight %}
</p>
      </div>
    </div>
  </div>
  <div class="tab-pane" id="confJSONCreateTabContent" role="tabpanel" arial-labelledby="confJSONCreateTab">
    <div class="card bg-light">
      <div class="card-body">
<p>
{% highlight json %}
{
"EnabledObservers": ["Encoder", "BodySensor", "KinematicInertial"],
"RunObservers": ["Encoder", "BodySensor", "KinematicInertial"],
"UpdateObservers": ["Encoder", "KinematicInertial"],

"Observers":
{
  "Encoder":
  {
    "UpdatePositionFrom": "estimator",
    "UpdateVelocityFrom": "estimator",
    "Log" : true
  },

  "BodySensor":
  {
    "UpdateFrom": "estimator",
    "FloatingBaseSensor": "FloatingBase"
  }
}
}
{% endhighlight %}
</p>
      </div>
    </div>
  </div>
</div>


Note that most of the configuration used in this example is already the default. The only required fields are `EnabledObservers`, `RunObservers` and `UpdateObservers`. Only use observer-specific configuration when needed. The above example will set-up the pipeline described at the begining of this tutorial. When running your controller, you should now see this line in the terminal, and the log file will contain `observers_...` entries.

```
Observers: Encoder (position=estimator,velocity=estimator) -> [BodySensor (sensor=FloatingBase,update=estimator)] -> KinematicInertial (cutoff=0.01)
```

If that's not the case, there is something wrong with your observers set-up. Please check that your configuration file is correct, and that all observers were successfully loaded.

## Visualizing the estimated robot

In Rviz, create a new robot with the following parameters:

- `Robot Description path: /real/robot_description`

# Creating your own Observer

All observers must inherit from `mc_observers::Observer` ([doc]({{site_base}}/mc_rtc/doxygen.html#a01940)) and implement the following `virtual` functions:

- `virtual void reset (const mc_control::MCController &ctl)` : reset the observers state
- `virtual bool run (const mc_control::MCController &ctl)` : estimate the state, but does not update the real robot instance
- `virtual void	updateRobots (const mc_control::MCController &ctl, mc_rbdyn::Robots &realRobots)` : update the robot state from the estimated state

To compile your own observer, you can use the provided macro

```cmake
add_observer(YourObserverName YourObserver.cpp YourObserver.h)
```
