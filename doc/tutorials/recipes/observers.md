---
layout: tutorials
toc: true
---

One commonly needs to know the real state of the robots being controlled. Unfortunately, their state can rarely be fully known, as the robots' embedded sensors rarely provide sufficient information. Instead, the systems' state relevant to the controller need to be inferred from various sensor measurements (joint encoders, force-torque sensors, IMU, cameras, etc), along with additional knowledge about the controller's intent (contacts, etc). For instance, a floating-base robot typically doesn't have any sensor capable of providing the full state of its floating base (position, orientation, velocity, etc), and one must make use of the available information to estimate it. This may for instance a combination of IMU, kalman filters and known information about the robot kinematics. Or one might choose to use visual odometry instead, obtain ground truth measurements from a motion capture system, or any combination of those. This process is refered to as state observation.

Each controller has its own requirements on what the state of the observed robots should be. A humanoid robot's walking controller will be particularly interesting in robustly estimating the state of the robot's center of mass, and thus needs to know the full kinematic state of the robot (position and velocity of each of its bodies, including the floating base), but a manipulator arm might only care about joint position and velocity. One may also be interested in comparing various methods to obtain that state (ground truth information vs estimated, different sensors or algorithms, etc).

The framework provides a mechanism, refered to as **State Observation Pipelines**, to simplify and generalize the observation of a robot, or multiple robots states. The concept is to view state observation as a pipelines, where each pipeline is composed of multiple observers executed sequentially. Each observer is a component responsible for estimating part of the robot state. When combined together, all observers in a pipeline contribute to provide a full estimation of the desired robot state. Multiple pipelines can be defined and executed allowing to estimate the state of multiple robots, or to perform comparisons between multiple estimation methods. The observers themselves are, as is the case for controller, tasks, plugins, loaded from libraries with a simple interface, allowing to conveniently define your own. The framework currently provides the following observers by default:

- **Encoder Observer**: estimates a robot's joint state (position and velocity) and computes forward kinematics and velocities to obtain body positions and velocities. Various inputs may be used: encoder position, encoder velocity (obtained from a velocity sensor or by finite differences of position), another robot's joint values, etc.
- **BodySensor Observer**: sets a robot's floating base state from measurements provided by a sensor attached to a robot body and the kinematics between the sensor and the floating base. This is typically be used to exploit ground truth measurement from a simulator, or exploit the results of an external component providing information about the floating base (MOCAP, embedded estimator on the robot plateform, etc).
- **KinematicInertial Observer**: estimates a robot's floating base pose (position + orientation) and velocity (low-pass filtered finite differences of position) from a {% doxygen mc_rbdyn::BodySensor %} (IMU orientation) and a kinematic anchor frame.

To represent the robots, the framework provides two sets of robot instances ({% doxygen mc_rbdyn::Robots %}):
- {% doxygen mc_control::MCController::robots() %} represents the control state (desired) of the robots.
- {% doxygen mc_control::MCController::realRobots() %} represents the state of the real robots. It's the role of the observer pipeline to define how these robot states are estimated.



# Configuring the observer pipelines

State observation pipelines can be configured in your controller configuration (each pipeline configuration superseeds the previous one):

- Global configuration: {% ihighlight bash %}$INSTALL_PREFIX/etc/mc_rtc.yaml{% endihighlight %}
- User configuration: {% ihighlight bash %}$HOME/.config/mc_rtc/mc_rtc.yaml{% endihighlight %}
- Controller-specific configuration: {% ihighlight bash %}$HOME/.config/mc_rtc/mc_controllers/YourController.yaml{% endihighlight %}
- The FSM configuration of your controller: {% ihighlight bash %} YouController.yaml{% endihighlight %} (recommended)

The configuration format is fully documented in the [Observers JSON schema](../../json.html#Observers/ObserverPipelines).

Let's look first at a simple representative example to estimate the state of a floating-base robot from encoder position measurements and a sensor providing roll and pitch orientation of the floating base.

```yaml
---
ObserverPipelines:
- name: MainPipeline                     # - Create a new pipeline
  gui: true                              #   diplay the pipeline in the GUI (default = false)
  log: true                              #   log observers (default)

  observers:                             #   declare which observers to use
  - type: Encoder                        # - Use an EncoderObserver
    config:                              #
      position: encoderValues            #    - Sets joint position from encoder sensor values (default)
      velocity: encoderFiniteDifferences #    - Computes joint velocities by finite differences  (default)
                                         # We now have the estimation of each joint position and velocity and the corresponding
                                         # body positions and velocities, but we are still missing the floating base

  - type: BodySensor                     # - Use a BodySensor observer
    update: false                        #   Do not update the real robot state
    gui: false                           #   Do not display in the gui
    config:                              #
      bodySensor: FloatingBase           #   In simulation, the interface will fill this sensor with ground truth values
                                         #   The observer computes the position and velocity of the floating base
                                         #   by transforming the sensor measurements to the floating base frame

  - type: KinematicInertial              # - Estimates the floating base state using the KinematicInertial observer
    update: true                         #   update the real robot instance from its results
    gui: true                            #   Displays the estimated velocity as an arrow (default)
    config:
      bodySensor: Accelerometer          # This observer only uses roll and pitch rotation information from this sensor
                                         # along with a kinematic anchor point and the robot kinematics between the anchor
                                         # frame and the floating base frame. The anchor frame is expected to be provided
                                         # through a datastore callback (see below for details)
```

Please refer to the corresponding JSON schemas for a full list of available options:
- [ObserverPipelines](../../json.html#Observers/ObserverPipelines): array of multiple state observation pipelines
- [ObserverPipeline](../../json.html#Observers/ObserverPipeline): definition of an observer pipeline, containing observers
- [EncoderObserver](../../json.html#Observers/Encoder): options for the encoder observer
- [BodySensorObserver](../../json.html#Observers/BodySensor): options for the BodySensor observer
- [KinematicInertial](../../json.html#Observers/KinematicInertial): options for the KinematicInertial observer

When creating a controller with the above pipeline, the framework will display a short summary as

```
ObserverPipelines:
- ExamplePipeline: Encoder (position=encoderValues,velocity=encoderFiniteDifferences) -> [BodySensor (sensor=FloatingBase,update=sensor)] ->  KinematicInertial (sensor=Accelerometer,cutoff=0.010000)
```

This displays information about the running pipelines, and their sequence of observers. Observers displayed between `[..]` brackets are run but do not affect the state of the `realRobots` instance. After running the pipelines, the `realRobots` instances now contain the estimated robot states, and can be used in the controller.

For instance, with the above pipeline you can:
- Get joint position `readRobot().mbc().q()` and velocity `realRobot().mbc().alpha()`
- Get the floating base pose `realRobot().posW()`
- Get the floating base velocity `realRobot().velW()`
- Get the pose of a body: `realRobot().bodyPosW("bodyName");`
- Get the velocity of a body: `realRobot().bodyVelW("bodyName");`
- Compute the CoM position and velocity `realRobot().com() / realRobot().comVelocity()`
- ...

The end-result for this example pipeline looks like this *(left: choreonoid simulation, right: control state [transparent], observed state [solid])*

<div class="embed-responsive embed-responsive-16by9">
  <iframe class="embed-responsive-item" src="https://www.youtube.com/embed/ssoNkV940yc" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

# Default observers

This section provides a brief description of the default observers provided with the framework. Please refer to each observer's API documentation and JSON Schema for further details.

## Encoder observer

- API: {% doxygen mc_observers::EncoderObserver %}
- [JSON Schema](../../json.html#Observers/Encoder)

The encoder observer may be used to obtain the position and velocity of all actuated joints.

- Joint values can be obtained from sensor as provided by {% doxygen mc_rbdyn::Robot::encoderValues() %} or using the joint state of another robot {% doxygen mc_rbdyn::Robot::q() %}.
- Joint velocities can be obtained from a joint velocity sensor {% doxygen mc_rbdyn::Robot::encoderVelocities() %}, estimated by finite differences of the (estimated) position, or using the joint velocities of another robot {% doxygen mc_rbdyn::Robot::alpha() %}
- The observer will compute forward kinematics {% doxygen mc_rbdyn::Robot::forwardKinematics() %} and forward velocity {% doxygen mc_rbdyn::Robot::forwardVelocity() %} to update the corresponding body positions and velocities (which may be required by subsequent observers).

## BodySensor observer

- API: {% doxygen mc_observers::BodySensorObserver %}
- [JSON Schema](../../json.html#Observers/BodySensor)

The BodySensor observer allows to estimate the state of the floating base from information provided by a {% doxygen mc_rbdyn::BodySensor %}. BodySensors are sensors attached to a robot body representing measurements of its state. A `BodySensor` may contain:

- Transformation between the sensor and the body to which it is attached [required]
- Position and orientation of the body
- Linear and angular velocity
- Linear and angular acceleration

Depending on the available sensors on the robot, only some of these measurements may be available while others will default to zero. The BodySensor observer requires at least the position, orientation, linear and angular velocity measurements. Futhermore, if the sensor is not directly attached to the floating base, it requires the kinematic transformation between the sensor and the floating-base to be up-to-date (you may for instance use the `EncoderObserver`).

This observer is commonly used to let simulation interfaces provide ground truth measurements of the floating base state through a `FloatingBase` body sensor.

## KinematicInertial observer

- API: {% doxygen mc_observers::KinematicInertialObserver %}
- [JSON Schema](../../json.html#Observers/KinematicInertial)
- [Detailed explanation of the method by Stéphane Caron](https://scaron.info/teaching/floating-base-estimation.html)

Floating base robots rarely have sensors capable of providing the full state of the floating base. The KinematicInertial observer provides a simple method to estimate the floating base position, orientation, linear and angular velocity based on measurement based on orientation estimation obtained from an IMU sensor. Please refer to [Stéphane Caron's website](https://scaron.info/teaching/floating-base-estimation.html) for a full explanation of the method implemented here. Note that this observer has been extensively used in practice in the [LIPM Walking Controller](https://github.com/jrl-umi3218/lipm_walking_controller) for walking, including stair climbing ([video](https://www.youtube.com/embed/vFCFKAunsYM)).

The method expects the roll and pitch rotation angle w.r.t gravity to be provided by a sensor. The rotation along the gravity vector (yaw) is assumed to be unobservable by sensor (as is the case for IMU), and is instead replaced by the desired rotation of the control robot to provide a complete estimation of the sensor's frame.

The estimation of position relies on the assumption that the contact position is known and only concerns itself with estimating the position that best matches the observed orientation. This is achieved by providing a anchor point in-between the assumed contact positions. When static a suitable choice of anchor frame is a point center in-between all contacts. When performing motions such as walking, the anchor frame is expected to be smoothly interpolated in-between the contacts such that there is no discontinuity when contact transition occurs. This frame is expected to be provided through a datastore callback [datastore]({{site.baseurl}}/tutorial/recipes/datastore.html).

For example, in the pipeline described in the previous section, the anchor frame is provided as a frame center between the two expected feet contact.

```cpp
double leftFootRatio = 0.5
ctl.datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                          [this, &leftFootRatio](const mc_rbdyn::Robot & robot)
                          {
                            return sva::interpolate(robot().surfacePose("LeftFoot"),
                                                    robot().surfacePose("RightFoot"),
                                                    leftFootRatio)
                          });
```

Note that this function will be called twice: once for the control robot instance, once for the real robot instance (only the relative pose between the anchor frame and the sensor frame is used by the observer).

If one wanted to walk, starting with a right contact and left swing foot motion, the anchor frame would need to be moved smoothly towards the right contact first until contact transition occurs, then be moved smoothly between the two planned contacts such that it ends up at the newly established left foot contact once the step is completed. Discontinuities in the anchor frame may result in jumps of the estimated floating base position (orientation is not affected).

This observer then computes the floating base linear and angular velocity by finite differences of the estimated position and orientation. This velocity is low-pass filtered.

You can find examples of this observer in use in the [Admittance sample controller tutorial]({{site.baseurl}}/tutorials/samples/sample-admittance.html), the {% doxygen mc_control::fsm::StabilizerStandingState %}, and a more complex use in the [LIPMWalking controller](
https://github.com/jrl-umi3218/lipm_walking_controller).

## Visualizing the estimated robot

You may visualize the estimated robots in rviz. By default the main robot instance is shown as the `RealRobot` element with the following properties.

```yaml
Robot Description path: /real/robot_description
TF Prefix: /real
```

Additional robots are published as:

```yaml
# robots published as env_1, env_2, etc
Robot Description path: /real/env_*/robot_description
TF Prefix: /real/env_*
```

# Interacting with observer pipelines from code

In some cases, it might be useful to interact with observer pipelines in your code. You might for instance want to:
- check whether a specific observation pipeline is running
- check whether some specific observers are present
- know the status of specific observers from code
- only perfom actions if some observers are present
- etc...

## Querying the status of observer pipelines

Here is a short snippet of code to showcase how one can query the status of observer pipelines and take actions accordingly:

```cpp
bool checkObserverPipeline(const std::string & observerPipelineName)
{
  if(!hasObserverPipeline(observerPipeline))
  {
    mc_rtc::log::error("This controller does not have a pipeline named {}", observerPipelineName);
    return false;
  }
  const auto & observerp = observerPipeline(observerPipelineName);
  if(!observerp.success()) // Check if the pipeline failed
  {
    mc_rtc::log::error("Required pipeline \"{}\" for real robot observation failed to run!", observerPipelineName);
    // Check which observer failed
    for(const auto & observer : observerp.observers())
    {
      if(!observer.success())
      {
        // Display failure error
        mc_rtc::log::error("Observer \"{}\" failed with error \"{}\"", observer.observer().name(), observer.observer().error());
        if(observer.observer.name() == "MyObserver")
        {
          // do something specific if this observer failed
        }
      }
    }
    return false;
  }
  return true;
}
```

Now calling `checkObserverPipeline("RequiredObserverPipeline");` will inform you if this pipeline does not exist, show exactly which observer is failing and why, and do something specific with `MyObserver` if it is in the pipeline.
For further available functionnalities, please refer to {% doxygen mc_observers::ObserverPipeline %} documentation.

# Creating your own Observer

The framework loads observers from libraries. For this to work, your observer must inherit from {% doxygen mc_observers::Observer %} and implement the following `virtual` functions:

```cpp
// YourObserver.h
void configure(const mc_control::MCController & /*ctl*/, const mc_rtc::Configuration & /*config*/) override
void reset (const mc_control::MCController &ctl) override
bool run (const mc_control::MCController &ctl) override
void update(mc_control::MCController &ctl) override
```

Furthermore, in order for `mc_rtc` to find your observer, you must define the loading symbols that `mc_rtc`'s observer loader will look for in your library. This can be done through the following macro defined in `mc_observers/ObserverMacros.h`

```cpp
// YourObserver.cpp
#include <mc_observers/ObserverMacros.h>
// Observer implementation (configure, reset, run and update functions)
EXPORT_OBSERVER_MODULE("YourObserver", your_namespace::YourObserverClassName)
```

To compile your own observer, you can use the provided macro, which takes care of linking your observer with `mc_observers`, and installing it in the default observers path.

```cmake
add_observer(YourObserverName YourObserver.cpp YourObserver.h)
```

Note: If you wish to inherit from one of the default observers provided along with the framework, you will need to link against it. For example, if you inherit from {% doxygen mc_observers::BodySensorObserver %} you need to link against the corresponding `mc_observers::BodySensorObserver %} target:

```cmake
target_link_libraries(YourObserverName PUBLIC mc_observers::BodySensorObserver)
```

For a practical example, please refer to this [sample project](https://github.com/arntanguy/mc_observer_example).
