[ROS](https://en.wikipedia.org/wiki/Robot_Operating_System), the Robot
Operating System, is a very popular middleware in robotics and many tools in
the domain are able to communicate (sometimes solely) using ROS. However, ROS
was not designed for real-time applications and thus this should be taken into
account when integrating ROS products with mc\_rtc. The remainder of this
document assumes the reader has some level of familiarity with ROS principles.

## What does mc\_rtc communicates to ROS?

Depending on the [options provided]({{site.baseurl}}/tutorials/introduction/configuration.html), mc\_rtc can publish:
1. the controller state of the robot, this it the state that mc\_rtc uses as a reference.
2. the real state of the robot, this is the state provided by the interface, it may be incomplete in some cases.

The first is published in the `control` namespace while the second is published in the `real` namespace.

Everything else is up to the controller code. In particular, <strong>mc\_rtc will not take care of spinning.</strong>

For external tools, the `mc_rtc::RobotPublisher` class is provided to publish a robot's state to ROS.

## Getting a NodeHandle

The following code will give you access to the `ros::NodeHandle` created by mc\_rtc:
```cpp
#include <mc_rtc/ros.h>

std::shared_ptr<ros::NodeHandle> nh = mc_rtc::ROSBridge::get_node_handle();
```

Note that the returned pointer is null if mc\_rtc was built without ROS support
or if ROS was unable to initialize (usually because the master is not
available).

## Good practices

#### Avoid ROS code in real-time parts

The main caveat when working with ROS (in mc\_rtc) is the absence of real-time
support. As such, you are strongly encourage not to use ROS functions within:
1. The `reset()` function
2. The `run()` function

Both these functions will occur in the real-time loop or real-time sensitive
context. A good scheme is to leave all ROS operations in a separate thread. The
synchronization issue is out of the scope of this document but C++11 and
further provide many tools to that effect.
